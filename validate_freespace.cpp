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

QWidget* createFreeSpaceValidationPlot() {
    // 1. Simulation Parameters
    constexpr double freq = 26e9; // 26 GHz
    constexpr double c = 3e8;
    constexpr double lambda = c / freq;

    constexpr double p_tx_dBm = 20.0;
    constexpr double g_tx_linear = 1.64; // lambda/2 dipole
    constexpr double g_rx_linear = 1.64;

    double g_tx_dB = 10.0 * std::log10(g_tx_linear);
    double g_rx_dB = 10.0 * std::log10(g_rx_linear);

    // 2. Generate Theoretical Data
    QLineSeries *friisSeries = new QLineSeries();
    friisSeries->setName("Friis Free-Space (n=2)");
    QPen friisPen(Qt::blue);
    friisPen.setWidth(2);
    friisSeries->setPen(friisPen);

    double max_dist = 100.0;
    int num_points = 500;
    double step = max_dist / num_points;

    double min_pwr = 0; // For dynamic Y-axis
    double max_pwr = -200;

    for (int i = 1; i <= num_points; i++) {
        double d = i * step;

        // FSPL Equation: PRX = PTX + GTX + GRX - 20*log10(4*pi*d / lambda)
        double fspl_dB = 20.0 * std::log10((4.0 * M_PI * d) / lambda);
        double p_rx_dBm = p_tx_dBm + g_tx_dB + g_rx_dB - fspl_dB;

        friisSeries->append(d, p_rx_dBm);

        if (p_rx_dBm < min_pwr) min_pwr = p_rx_dBm;
        if (p_rx_dBm > max_pwr) max_pwr = p_rx_dBm;
    }

    // 3. Build the Chart
    QChart *chart = new QChart();
    chart->addSeries(friisSeries);
    chart->setTitle("Validation: Free-Space Propagation (26 GHz)");

    // X-Axis (Logarithmic to match course slide 26)
    QLogValueAxis *axisX = new QLogValueAxis();
    axisX->setTitleText("Distance (m) [Log Scale]");
    axisX->setBase(10.0);
    axisX->setRange(1.0, 100.0);
    axisX->setMinorTickCount(8);
    axisX->setLabelFormat("%g");
    chart->addAxis(axisX, Qt::AlignBottom);
    friisSeries->attachAxis(axisX);

    // Y-Axis
    QValueAxis *axisY = new QValueAxis();
    axisY->setTitleText("Received Power (dBm)");
    axisY->setRange(std::floor(min_pwr / 10.0) * 10.0, std::ceil(max_pwr / 10.0) * 10.0);
    chart->addAxis(axisY, Qt::AlignLeft);
    friisSeries->attachAxis(axisY);

    // 3. Create the UI Window
    QChartView *chartView = new QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);

    QWidget *window = new QWidget();
    window->setWindowTitle("Validation: Free-Space");
    window->resize(800, 600);
    window->setAttribute(Qt::WA_DeleteOnClose);

    QVBoxLayout *mainLayout = new QVBoxLayout(window);
    mainLayout->addWidget(chartView, 1);

    QHBoxLayout *buttonLayout = new QHBoxLayout();
    QPushButton *btnSavePng = new QPushButton("Save Chart PNG");
    QPushButton *btnScene = new QPushButton("View 2D Scene Snapshot (d=50m)");
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

    return window;
}

