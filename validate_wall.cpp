#include "validate_wall.h"
#include "validate_common_utils.h"

#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QFileDialog>
#include <QLogValueAxis>
#include <QPushButton>
#include <QVBoxLayout>
#include <cmath>
#include <complex>

QWidget* createTwoRayValidationPlot() {
    // 1. Simulation Parameters
    constexpr double freq = 26e9; // 26 GHz
    constexpr double c = 3e8;
    constexpr double lambda = c / freq;
    constexpr double beta = (2.0 * M_PI) / lambda;

    constexpr double p_tx_dBm = 20.0;
    constexpr double g_tx_linear = 1.64;
    constexpr double g_rx_linear = 1.64;

    double g_tx_dB = 10.0 * std::log10(g_tx_linear);
    double g_rx_dB = 10.0 * std::log10(g_rx_linear);

    // Environment: Wall is parallel to the TX-RX line
    constexpr double epsilon_r = 4.0; // Relative permittivity of the wall
    constexpr double w = 5.0; // Perpendicular distance from TX/RX to the wall (meters)

    // 2. Generate Theoretical Data
    QLineSeries *twoRaySeries = new QLineSeries();
    twoRaySeries->setName("Two-Ray Interference (Wall at 5m)");
    QPen twoRayPen(Qt::red);
    twoRayPen.setWidth(2);
    twoRaySeries->setPen(twoRayPen);

    QLineSeries *friisSeries = new QLineSeries(); // Baseline for comparison
    friisSeries->setName("Friis Free-Space (n=2)");
    QPen friisPen(Qt::blue);
    friisPen.setStyle(Qt::DashLine);
    friisSeries->setPen(friisPen);

    double max_dist = 100.0;
    int num_points = 2000; // High resolution needed to capture high-frequency 26 GHz fringes
    double step = max_dist / num_points;

    for (int i = 1; i <= num_points; i++) {
        double d_horizontal = i * step; // x-axis separation

        // Ray 1: Direct Path
        double d1 = d_horizontal;
        std::complex<double> E1 = std::polar(1.0 / d1, -beta * d1);

        // Ray 2: Reflected Path (Image Method)
        double d2 = std::sqrt(std::pow(d_horizontal, 2) + std::pow(2.0 * w, 2));

        // Angle of incidence with the normal of the wall
        double theta_i = std::atan(d_horizontal / (2.0 * w));

        // Perpendicular reflection coefficient (assuming vertical dipoles, E-field is normal to incidence plane)
        double cos_theta_i = std::cos(theta_i);
        double sin_theta_i = std::sin(theta_i);
        double gamma = (cos_theta_i - std::sqrt(epsilon_r - std::pow(sin_theta_i, 2))) /
                       (cos_theta_i + std::sqrt(epsilon_r - std::pow(sin_theta_i, 2)));

        std::complex<double> E2 = std::polar(gamma / d2, -beta * d2);

        // Complex sum of the E-field phasors
        std::complex<double> E_total = E1 + E2;

        // Power calculation: std::norm returns the squared magnitude |z|^2
        double power_factor = std::norm(E_total);

        // P_RX = P_TX * G_TX * G_RX * (lambda / 4*pi)^2 * |E1 + E2|^2
        double p_rx_dBm = p_tx_dBm + g_tx_dB + g_rx_dB +
                          20.0 * std::log10(lambda / (4.0 * M_PI)) +
                          10.0 * std::log10(power_factor);

        twoRaySeries->append(d_horizontal, p_rx_dBm);

        // Compute baseline FSPL for plotting
        double p_rx_friis = p_tx_dBm + g_tx_dB + g_rx_dB - 20.0 * std::log10((4.0 * M_PI * d_horizontal) / lambda);
        friisSeries->append(d_horizontal, p_rx_friis);
    }

    // 3. Build the Chart
    QChart *chart = new QChart();
    chart->addSeries(twoRaySeries);
    chart->addSeries(friisSeries);
    chart->setTitle("Validation: Single Reflection Path (26 GHz)");
    QFont titleFont = chart->titleFont();
    titleFont.setPointSize(16);
    titleFont.setBold(true);
    chart->setTitleFont(titleFont);

    // X-Axis (Linear to clearly see the spatial frequency of the fringes)
    QValueAxis *axisX = new QValueAxis();
    axisX->setTitleText("Distance (m)");
    axisX->setRange(1.0, 100.0);
    axisX->setTickCount(11);
    chart->addAxis(axisX, Qt::AlignBottom);
    friisSeries->attachAxis(axisX);
    twoRaySeries->attachAxis(axisX);

    // Y-Axis
    QValueAxis *axisY = new QValueAxis();
    axisY->setTitleText("Received Power (dBm)");
    axisY->setRange(-110.0, -30.0); // Fixed bounds to clearly show the deep fades
    chart->addAxis(axisY, Qt::AlignLeft);
    friisSeries->attachAxis(axisY);
    twoRaySeries->attachAxis(axisY);

    // 3. Create the UI Window
    QChartView *chartView = new QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);

    QWidget *window = new QWidget();
    window->setWindowTitle("Validation: Two-Ray");
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
        double scale = 10.0;
        double w = 5.0 * scale; // Wall at Y = 5m
        double d = 50.0 * scale;

        // Wall
        scene->addLine(0, w, d, w, QPen(Qt::black, 3));

        // TX and RX
        scene->addEllipse(-5, -5, 10, 10, QPen(Qt::red), QBrush(Qt::red));
        scene->addEllipse(d-5, -5, 10, 10, QPen(Qt::blue), QBrush(Qt::blue));

        // Direct Ray
        scene->addLine(0, 0, d, 0, QPen(Qt::green, 1, Qt::DashLine));

        // Reflected Ray (Specular point is exactly at d/2 due to symmetry)
        scene->addLine(0, 0, d/2, w, QPen(Qt::magenta, 2));
        scene->addLine(d/2, w, d, 0, QPen(Qt::magenta, 2));

        scene->setSceneRect(-20, -20, d + 40, w + 40);
        showValidationScene(scene, "Single Reflection Geometry Snapshot (d=50m)");
    });

    QObject::connect(btnToggleAxis, &QPushButton::clicked, [chart, twoRaySeries, friisSeries]() {
        QAbstractAxis *oldAxisX = chart->axes(Qt::Horizontal).constFirst();
        bool isLog = (oldAxisX->type() == QAbstractAxis::AxisTypeLogValue);

        twoRaySeries->detachAxis(oldAxisX);
        friisSeries->detachAxis(oldAxisX);

        chart->removeAxis(oldAxisX);
        delete oldAxisX;

        if (isLog) {
            QValueAxis *newAxisX = new QValueAxis();
            newAxisX->setTitleText("Distance (m)");
            newAxisX->setRange(1.0, 100.0);
            newAxisX->setTickCount(11);
            chart->addAxis(newAxisX, Qt::AlignBottom);
            twoRaySeries->attachAxis(newAxisX);
            friisSeries->attachAxis(newAxisX);
        } else {
            QLogValueAxis *newAxisX = new QLogValueAxis();
            newAxisX->setTitleText("Distance (m) [Log Scale]");
            newAxisX->setBase(10.0);
            newAxisX->setRange(1.0, 100.0);
            newAxisX->setMinorTickCount(8);
            newAxisX->setLabelFormat("%g");
            chart->addAxis(newAxisX, Qt::AlignBottom);
            twoRaySeries->attachAxis(newAxisX);
            friisSeries->attachAxis(newAxisX);
        }
    });

    return window;
}

