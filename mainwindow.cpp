#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QDesktopServices>
#include <QProcess>
#include <stdio.h>
#include <QMessageBox>
#include <QFileDialog>
#include <QDir>
#include <QChartView>

#include "parameters.h"

#include "simulation.h"
//#include "validation.h" // old_validation
#include "validation_free_space.h"
#include "validation_single_reflection.h"
#include "validation_double_reflection.h"
#include "validation_triple_reflection.h"
//#include "validation_recursive.h"

#include "validate_freespace.h"
#include "validate_wall.h"
#include "validate_corridor.h"
QWidget* Validate_view;

QGraphicsView* Validation_view;
QChartView* plot_view;

Simulation simulation = Simulation(); // The global simulation object, use `extern Simulation simulation;` in other files?

int currentEditingBaseStation_index = 0; // The base station that is currently selected for user edit

QList<qreal> resolutions = {1.0, 0.5, 0.25, 0.125};//, 0.0625};

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    // MainWindow constructor, is ran on program's UI launch
    this->setWindowIcon(QIcon(":/assets/icon.png"));
    initFirstBaseStation();
    ui->setupUi(this);
    ui->baseStationXspinBox->setMaximum(max_x);
    ui->baseStationYspinBox->setMaximum(max_y);
    ui->cellXspinBox->setMaximum(max_x);
    ui->cellYspinBox->setMaximum(max_y);

    showFirstBaseStation();

    toggleCoverageParametersLayout(true);
    toggleCellParametersLayout(false);

    for (qreal resolution : std::as_const(resolutions)) {
        ui->resolutionComboBox->addItem(QString("%1 m").arg(resolution));
    }
    ui->resolutionComboBox->setCurrentIndex(0);

    this->setWindowIcon(QIcon(":/assets/icon.png"));
}

MainWindow::~MainWindow()
{
    // MainWindow destructor
    delete ui;
}

void MainWindow::on_runSimulationButton_clicked()
{
    // User clicked on the "Run Simulation" button

    if (simulation.is_running) {
        return;
    }

    qInfo("\n### Starting simulation ###");
    QProgressBar* progress_bar = ui->simulationProgressBar;
    progress_bar->setValue(100);
    ui->runSimulationButton->setEnabled(false);
    ui->runSimulationButton->setStyleSheet("font-size: 14px;");
    ui->runSimulationButton->setText("[Menu->Reset App] or (Ctrl+R) to reset");
    bool res = runSimulation(progress_bar);
    progress_bar->setValue(100);
    // Turn the text green if simulation ran successfully, red otherwise
    ui->runSuccessOrFailText->setStyleSheet(res ? "color: green;" : "color: red;");
    ui->runSuccessOrFailText->setText(res ? QString("Success, took %1ms").arg(simulation.getSimulationTime()): "Failed");

    simulation.is_running = false;
}

bool MainWindow::runSimulation(QProgressBar* progress_bar)
{

    // Run the simulation and returns true if no errors ocurred
    simulation.ran = true;
    simulation.is_running = true;
    try {
        simulation.run(progress_bar);

        qInfo("### Simulation ended successfully ###\n");
        return true;
    } catch (...) {
        qInfo("#!#!#!# Simulation failed #!#!#!#\n");
        return false;
    }
}

void MainWindow::changeBaseStationPower(qreal value)
{
    // Modify the current editing base station with the new user chosen power
    qDebug() << "dBm value:" << value;
    //Transmitter* base_station = simulation.getBaseStation(currentEditingBaseStation_index);
    Transmitter* base_station = simulation.baseStations[currentEditingBaseStation_index];

    //qreal power = pow(10.0, value / 10.0);
    //base_station->power=power;

    base_station->setPower_dBm(value);
    simulation.baseStations[currentEditingBaseStation_index]->power_dBm=value;
    
    //base_station->setPower_dBm(value);
    
    showBaseStationPower(value);
}

void MainWindow::on_sliderBaseStationPower_valueChanged(int value)
{
    // User changed the current editing base station power (with slider)
    changeBaseStationPower(value);
}

void MainWindow::on_spinBoxBaseStationPower_valueChanged(int value)
{
    // User changed the current editing base station power (with spin box)
    changeBaseStationPower(value);
}

void MainWindow::changeBaseStationCoordinates(QPointF point)
{
    // Modify the current editing base station with the new user chosen coordinates
    Transmitter* base_station = simulation.baseStations.at(currentEditingBaseStation_index);
    base_station->changeCoordinates(point);
    showBaseStationCoordinates(point);
}

void MainWindow::on_baseStationXspinBox_valueChanged(double x)
{
    // User changed the current editing base station X coordinate
    Transmitter* base_station = simulation.baseStations.at(currentEditingBaseStation_index);
    QPointF coordinates = base_station->toPointF();
    coordinates.setX(x);
    changeBaseStationCoordinates(coordinates);
}

void MainWindow::on_baseStationYspinBox_valueChanged(double y)
{
    // User changed the current editing base station Y coordinate
    Transmitter* base_station = simulation.baseStations.at(currentEditingBaseStation_index);
    QPointF coordinates = base_station->toPointF();
    coordinates.setY(y);
    changeBaseStationCoordinates(coordinates);
}

void MainWindow::on_actionExit_triggered()
{
    // User clicked on the menu's "Exit" button, or presse "CTRL+W"
    printf("Closing app...\n");
    qApp->quit();
}


void MainWindow::on_actionReset_triggered()
{
    // User clicked on the menu's "Reset" button, or presse "CTRL+R"
    qInfo("Resetting app...");

    qApp->quit();
    QProcess::startDetached(qApp->arguments()[0], qApp->arguments());
}


void MainWindow::on_actionSee_Github_triggered()
{
    // User clicked on the menu's "See Github" button
    QDesktopServices::openUrl(QUrl("https://github.com/LucasPlacentino/5G_channel_modeling_raytracing", QUrl::TolerantMode));
}


void MainWindow::on_actionAbout_triggered()
{
    // User clicked on the menu's "About" button
    QMessageBox::about(this, tr("About this application"),
                       tr("This <b>5G Channel Modeling Ray-Tracing Simulator</b> was made as a "
                          "project for the course <b>ELEC-H415</b> Communication Channels "
                          "at <b>École polytechnique de Bruxelles - ULB (BRUFACE Master program)</b>."
                          "<br>By Lucas Placentino, 2026"));
}

void MainWindow::saveImage(QGraphicsView* target_view, bool isValidation) // bool isValidation not used
{
    //// Resolve default argument
    //if (target_view == nullptr) {
    //    if (simulation.view == nullptr) return;
    //    target_view = simulation.view;
    //}
    if (target_view == nullptr) {
        qWarning() << "No view, cannot save image.";
        return;
    }

    if (!target_view || !target_view->scene()) {
        qWarning() << "No view or scene available to save!";
        return;
    }

    // 1. Ask for file location FIRST
    QString img_filename = QFileDialog::getSaveFileName(
        this,
        tr("Save Simulation Image"),
        QDir::currentPath(),
        "PNG (*.png);;BMP Files (*.bmp);;JPEG (*.jpg)"
        );

    if (img_filename.isEmpty()) {
        qInfo() << "Save Cancelled";
        return; // Abort before wasting CPU
    }

    // 2. Setup Image (Format_ARGB32_Premultiplied is much faster for Qt drawing than RGBA64)
    QGraphicsScene* scene = target_view->scene();
    QSize size = scene->sceneRect().size().toSize() * 10;
    QImage img(size, QImage::Format_ARGB32_Premultiplied);

    // 3. Clear the garbage memory!
    img.fill(Qt::black); // Use black since your UI uses a black background

    // 4. Paint the SCENE, not the view
    QPainter painter(&img);
    painter.setRenderHint(QPainter::Antialiasing);

    // Pass the target rect (the image) and source rect (the scene) to ensure perfect scaling
    scene->render(&painter, QRectF(img.rect()), scene->sceneRect());
    painter.end(); // Always end the painter before saving

    // 5. Save
    bool success = img.save(img_filename);
    if (success) {
        qInfo() << "Image successfully saved to:" << img_filename;
    } else {
        qWarning() << "Failed to save image!";
    }
}

void MainWindow::on_actionSave_image_triggered()
{
    // User clicked on the menu's "Save Image" button, captures an image of the simulation view.

    if (simulation.scene == nullptr || !simulation.ran) { // checks if a simulation has run
        qInfo("Cannot save simulation that has not already run.");
        //throw std::runtime_error("Cannot save simulation that has not already run.");
        return;
    }
    saveImage(simulation.view);
}


void MainWindow::initFirstBaseStation()
{
    // Creates the first (non-deletable) Base Station
    simulation.baseStations.append(new Transmitter(initial_BS_x,initial_BS_y, 0, "Base Station 1"));
}

void MainWindow::showFirstBaseStation()
{
    // Makes sure the first (non-deletable) Base Station is shown on the base station user edit box
    QString new_item = QString("Base Station 1");
    ui->transmitterSelector->addItem(new_item);
    ui->transmitterSelector->setCurrentIndex(0);
    on_transmitterSelector_activated(0);
}

void MainWindow::showBaseStationPower(int value_dBm)
{
    // Updates the UI to show this base station power
    ui->spinBoxBaseStationPower->setValue(value_dBm);
    ui->sliderBaseStationPower->setValue(value_dBm);
}

void MainWindow::showBaseStationCoordinates(QPointF point)
{
    // Updates the UI to show these base station coordinates
    ui->baseStationXspinBox->setValue(point.x());
    ui->baseStationYspinBox->setValue(point.y());
}

void MainWindow::toggleCellParametersLayout(bool enabled)
{
    //ui->cellParametersLayout->setEnabled(enabled);
    ui->cellCoordinatesLabel->setEnabled(enabled);
    ui->cellCoordinatesXLabel->setEnabled(enabled);
    ui->cellCoordinatesYLabel->setEnabled(enabled);
    ui->cellXspinBox->setEnabled(enabled);
    ui->cellYspinBox->setEnabled(enabled);
}

void MainWindow::toggleCoverageParametersLayout(bool enabled)
{
    ui->showCellOutlineCheckBox->setEnabled(enabled);
}

void MainWindow::on_transmitterSelector_activated(int index)
{
    // User selected a base station in the dropdown
    qDebug() << "Selected base station: " << index+1;

    // set current editing transmitter to this one
    currentEditingBaseStation_index = index;
    showBaseStationPower(simulation.getBaseStation(currentEditingBaseStation_index)->getPower_dBm());

    showBaseStationCoordinates(simulation.baseStations.at(currentEditingBaseStation_index)->toPointF());
}


void MainWindow::on_addTransmitterButton_clicked()
{
    // User clicked "Add Base Station" button
    if (ui->transmitterSelector->count() < ui->transmitterSelector->maxCount()) //check if has not reached the max number of transmitters
    {
        qDebug("Added base station");

        QString new_item_name = QString("Base Station %1").arg(ui->transmitterSelector->count()+1);
        ui->transmitterSelector->addItem(new_item_name);

        int new_item_index = ui->transmitterSelector->findText(new_item_name);

        simulation.baseStations.append(new Transmitter(initial_BS_x,initial_BS_y, new_item_index, new_item_name));

        on_transmitterSelector_activated(new_item_index);
        ui->transmitterSelector->setCurrentIndex(new_item_index);


    } else {
        qInfo("There is already the maximum number of base stations");
    }
}


void MainWindow::on_deleteBaseStationPushButton_clicked()
{
    // User clicked on the "Delete Base Station" button
    //currentEditingBaseStation_index
    if (ui->transmitterSelector->count()>1 && currentEditingBaseStation_index != 0)
    {
        ui->transmitterSelector->removeItem(currentEditingBaseStation_index);
        simulation.deleteBaseStation(currentEditingBaseStation_index);

        int previous_item_index = currentEditingBaseStation_index-1;
        on_transmitterSelector_activated(previous_item_index);
        ui->transmitterSelector->setCurrentIndex(previous_item_index);
    } else {
        qDebug("Cannot delete Base Station 1");
    }
}

void MainWindow::on_coverageHeatmapRadioButton_clicked(bool checked)
{
    simulation.showRaySingleCell = !checked;
    toggleCoverageParametersLayout(checked);
    toggleCellParametersLayout(!checked);
    qDebug() << "Coverage parameters" << (checked? "enabled," : "disabled,") << "Cell parameters" << (!checked? "enabled" : "disabled");
}


void MainWindow::on_singleCellRadioButton_clicked(bool checked)
{
    simulation.showRaySingleCell = checked;
    toggleCellParametersLayout(checked);
    toggleCoverageParametersLayout(!checked);
    qDebug() << "Coverage parameters" << (!checked? "enabled," : "disabled,") << "Cell parameters" << (checked? "enabled" : "disabled");
}


void MainWindow::on_liftOnFloorCheckBox_clicked(bool checked)
{
    qDebug() << "Set lift:" << checked;
    simulation.lift_is_on_floor = checked;
}

void MainWindow::on_showCellOutlineCheckBox_toggled(bool checked)
{
    simulation.show_cell_outline = checked;
    qDebug() << "Show cell outline:" << checked;
}


void MainWindow::on_resolutionComboBox_currentIndexChanged(int index)
{
    simulation.resolution = resolutions[index];
    qInfo() << "Changed resolution to" << resolutions[index] << "m";
}


// void MainWindow::on_actionRun_Validation_Simulation_triggered()
// {
//     Validation_view = runValidation();
// }


void MainWindow::on_actionSave_Validation_image_triggered()
{
    saveImage(Validation_view);
}

void MainWindow::on_cellXspinBox_valueChanged(double x)
{
    simulation.singleCellX = x;
}


void MainWindow::on_cellYspinBox_valueChanged(double y)
{
    simulation.singleCellY = y;
}

void MainWindow::on_actionRun_Free_Space_Validation_triggered()
{
    //Validation_view = runFreeSpaceValidation();
    qDebug() << "Running Free Space Validation...";
    Validate_view = createFreeSpaceValidationPlot();
    Validate_view->show();
    //runFreeSpaceValidation();
}


void MainWindow::on_actionRun_Single_Reflection_Validation_triggered()
{
    //Validation_view = runSingleReflectionValidation();
    qDebug() << "Running Single Reflection Validation...";
    Validate_view = createTwoRayValidationPlot();
    Validate_view->show();
    //runSingleReflectionValidation();
}


void MainWindow::on_actionRun_Double_Reflection_Validation_triggered()
{
    qDebug() << "Running Double Reflection Validation...";
    //Validation_view = runDoubleReflectionValidation();
}


void MainWindow::on_actionRun_Triple_Reflection_Validation_triggered()
{
    //Validation_view = runTripleReflectionValidation();
    qDebug() << "Running Triple Reflection (Corridor) Validation...";
    Validate_view = createThreeReflectionValidationPlot();
    Validate_view->show();
}


void MainWindow::on_actionRun_Recursive_Validation_triggered()
{
    qDebug() << "not implemented";
    //Validation_view = runRecursiveValidation();
}


void MainWindow::on_removeAllWallsCheckBox_toggled(bool checked)
{
    simulation.remove_all_walls = checked;
    qDebug() << "Remove all walls:" << checked;
}


void MainWindow::on_actionView_path_loss_plot_triggered()
{
    if (simulation.ran && simulation.baseStations.length() == 1) { // sim ran and only 1 BS
        qDebug() << "Showing path loss plot";
        plot_view = simulation.showPathLossScatterPlot();
    } else {
        qDebug() << "Simulation hasn't run yet! Cannot show path loss plot.";
    }
}

