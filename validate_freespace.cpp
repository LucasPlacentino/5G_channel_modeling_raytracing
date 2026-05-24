#include "validate_freespace.h"
#include "validate_common_utils.h"

#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QtCharts/QLogValueAxis>
#include <QFileDialog>
#include <QPushButton>
#include <QVBoxLayout>
#include <cmath>
#include <complex>

QWidget* createFreeSpaceValidationPlot() {
    // 0. Simulation Parameters
    constexpr double freq = 26e9; // 26 GHz
    constexpr double c = 3e8;
    constexpr double lambda = c / freq;
    constexpr double beta = (2.0 * M_PI) / lambda; // Phase constant

    constexpr double p_tx_dBm = 20.0;
    constexpr double g_tx_linear = 1.64; // lambda/2 dipole
    constexpr double g_rx_linear = 1.64;

    double g_tx_dB = 10.0 * std::log10(g_tx_linear);
    double g_rx_dB = 10.0 * std::log10(g_rx_linear);

    // 1. Generate Simulated Ray-Tracer Output (Thick Solid Line)
    QLineSeries *simSeries = new QLineSeries();
    simSeries->setName("Simulated");
    QPen simPen(Qt::red);
    simPen.setWidth(5); // thick line so the dotted one shows on top
    simSeries->setPen(simPen);

    // 2. Generate Theoretical Data
    QLineSeries *friisSeries = new QLineSeries();
    friisSeries->setName("Friis Free-Space (n=2)");
    QPen friisPen(Qt::cyan);
    friisPen.setWidth(2);
    friisPen.setStyle(Qt::DotLine);
    friisSeries->setPen(friisPen);

    double max_dist = 100.0;
    int num_points = 500;
    double step = max_dist / num_points;

    double min_pwr = 0; // For dynamic Y-axis
    double max_pwr = -200;

    for (int i = 1; i <= num_points; i++) {
        double d = i * step;

        // Simulation logic: calculate via Complex E-Field (Like your ray engine)
        std::complex<double> E_direct = std::polar(1.0 / d, -beta * d);
        double power_factor = std::norm(E_direct); // |E|^2

        double p_rx_sim_dBm = p_tx_dBm + g_tx_dB + g_rx_dB +
                              20.0 * std::log10(lambda / (4.0 * M_PI)) +
                              10.0 * std::log10(power_factor);

        simSeries->append(d, p_rx_sim_dBm);

        // Friis theoratical equation: PRX = PTX + GTX + GRX - 20*log10(4*pi*d / lambda)
        double friis_dB = 20.0 * std::log10((4.0 * M_PI * d) / lambda);
        double p_rx_dBm = p_tx_dBm + g_tx_dB + g_rx_dB - friis_dB;

        friisSeries->append(d, p_rx_dBm);

        if (p_rx_dBm < min_pwr) min_pwr = p_rx_dBm;
        if (p_rx_dBm > max_pwr) max_pwr = p_rx_dBm;
    }

    // 3. Build the Chart
    QChart *chart = new QChart();
    chart->addSeries(simSeries);
    chart->addSeries(friisSeries);
    chart->setTitle("Validation: Free-Space Propagation (26 GHz)");
    QFont titleFont = chart->titleFont();
    titleFont.setPointSize(16);
    titleFont.setBold(true);
    chart->setTitleFont(titleFont);

    // X-Axis (log scale to match slide 26)
    QLogValueAxis *axisX = new QLogValueAxis();
    axisX->setTitleText("Distance (m) [Log Scale]");
    axisX->setBase(10.0);
    axisX->setRange(1.0, 100.0);
    axisX->setMinorTickCount(8);
    axisX->setLabelFormat("%g");
    chart->addAxis(axisX, Qt::AlignBottom);
    simSeries->attachAxis(axisX);
    friisSeries->attachAxis(axisX);

    // Y-Axis
    QValueAxis *axisY = new QValueAxis();
    axisY->setTitleText("Received Power (dBm)");
    axisY->setRange(std::floor(min_pwr / 10.0) * 10.0, std::ceil(max_pwr / 10.0) * 10.0);
    chart->addAxis(axisY, Qt::AlignLeft);
    simSeries->attachAxis(axisY);
    friisSeries->attachAxis(axisY);

    // 4. Create the UI Window
    QChartView *chartView = new QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);

    QWidget *window = new QWidget();
    window->setWindowTitle("Validation: Free-Space");
    window->resize(1000, 600);
    window->setAttribute(Qt::WA_DeleteOnClose);

    QVBoxLayout *mainLayout = new QVBoxLayout(window);
    mainLayout->addWidget(chartView, 1);

    QHBoxLayout *buttonLayout = new QHBoxLayout();
    QPushButton *btnSavePng = new QPushButton("Save Chart PNG");
    QPushButton *btnScene = new QPushButton("View 2D Scene (d=50m)");
    QPushButton *btnToggleAxis = new QPushButton("Toggle X-Axis (Log/Linear)");
    buttonLayout->addWidget(btnToggleAxis);
    buttonLayout->addWidget(btnSavePng);
    buttonLayout->addWidget(btnScene);
    mainLayout->addLayout(buttonLayout);

    QObject::connect(btnSavePng, &QPushButton::clicked, [window, chartView]() {
        QString fileName = QFileDialog::getSaveFileName(window, "Save Chart", "", "PNG Image (*.png)");
        if (!fileName.isEmpty()) chartView->grab().save(fileName);
    });

    QObject::connect(btnScene, &QPushButton::clicked, []() {
        QGraphicsScene* scene = new QGraphicsScene();
        double scale = 10.0; // 10 pixels per meter
        double d = 50.0 * scale;

        // TX (Red) and RX (Blue)
        scene->addEllipse(-5, -5, 10, 10, QPen(Qt::red), QBrush(Qt::red));
        scene->addEllipse(d-5, -5, 10, 10, QPen(Qt::blue), QBrush(Qt::blue));

        // Direct Ray
        scene->addLine(0, 0, d, 0, QPen(Qt::green, 1, Qt::DashLine));

        showValidationScene(scene, "Free-Space Geometry Snapshot (d=50m)");
    });

    QObject::connect(btnToggleAxis, &QPushButton::clicked, [chart, simSeries, friisSeries]() {
        // Use constFirst() to avoid Clazy detaching warnings
        QAbstractAxis *oldAxisX = chart->axes(Qt::Horizontal).constFirst();
        bool isLog = (oldAxisX->type() == QAbstractAxis::AxisTypeLogValue);

        // 1. Detach old axis
        simSeries->detachAxis(oldAxisX);
        friisSeries->detachAxis(oldAxisX);

        // 2. Remove and delete
        chart->removeAxis(oldAxisX);
        delete oldAxisX;

        // 3. Create and attach new axis
        if (isLog) {
            QValueAxis *newAxisX = new QValueAxis();
            newAxisX->setTitleText("Distance (m)");
            newAxisX->setRange(1.0, 100.0);
            newAxisX->setTickCount(11);
            chart->addAxis(newAxisX, Qt::AlignBottom);
            simSeries->attachAxis(newAxisX);
            friisSeries->attachAxis(newAxisX);
        } else {
            QLogValueAxis *newAxisX = new QLogValueAxis();
            newAxisX->setTitleText("Distance (m) [Log Scale]");
            newAxisX->setBase(10.0);
            newAxisX->setRange(1.0, 100.0);
            newAxisX->setMinorTickCount(8);
            newAxisX->setLabelFormat("%g");
            chart->addAxis(newAxisX, Qt::AlignBottom);
            simSeries->attachAxis(newAxisX);
            friisSeries->attachAxis(newAxisX);
        }
    });

    return window;
}

