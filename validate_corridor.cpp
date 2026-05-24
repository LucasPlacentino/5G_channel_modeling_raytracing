#include "validate_corridor.h"
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

// Computes the Dielectric Slab Reflection Coefficient (Gamma_m)
std::complex<double> calcDielectricSlabReflection(double theta_i, double freq, double epsilon_r, double thickness) {
    constexpr double c = 3e8;
    double lambda = c / freq;
    double beta = (2.0 * M_PI) / lambda;

    double cos_theta = std::cos(theta_i);
    double sin_theta = std::sin(theta_i);

    // 1. Single interface perpendicular reflection coefficient
    double root_term = std::sqrt(epsilon_r - std::pow(sin_theta, 2));
    double gamma_perp = (cos_theta - root_term) / (cos_theta + root_term);

    // 2. Phase delay inside the slab (q)
    double q = beta * thickness * root_term;

    // 3. Infinite internal reflections sum (Slab Coefficient)
    std::complex<double> j(0.0, 1.0);
    std::complex<double> exp_term = std::exp(-2.0 * j * q);

    std::complex<double> gamma_m = gamma_perp * (1.0 - exp_term) / (1.0 - (std::pow(gamma_perp, 2) * exp_term));

    return gamma_m;
}

QWidget* createThreeReflectionValidationPlot() {
    // 1. Simulation Parameters
    constexpr double freq = 26e9; // 26 GHz
    constexpr double c = 3e8;
    constexpr double lambda = c / freq;
    constexpr double beta = (2.0 * M_PI) / lambda;

    constexpr double p_tx_dBm = 20.0;
    double g_tx_dB = 10.0 * std::log10(1.64); // lambda/2 dipole
    double g_rx_dB = 10.0 * std::log10(1.64);

    // Corridor Geometry & Material
    constexpr double epsilon_r = 4.0;
    constexpr double wall_thickness = 0.40; // 40cm concrete slab
    constexpr double W = 5.0; // Distance from centerline to each wall (10m wide corridor)

    // 2. Generate Theoretical Data
    QLineSeries *corridorSeries = new QLineSeries();
    corridorSeries->setName("Corridor/Canyon (up to 3 reflections)");
    QPen corridorPen(Qt::red);
    corridorPen.setWidth(2);
    corridorSeries->setPen(corridorPen);

    QLineSeries *friisSeries = new QLineSeries();
    friisSeries->setName("Friis Free-Space (n=2)");
    QPen friisPen(Qt::blue);
    friisPen.setStyle(Qt::DashLine);
    friisSeries->setPen(friisPen);

    double max_dist = 200.0;
    int num_points = 4000;
    double step = max_dist / num_points;

    for (int i = 1; i <= num_points; i++) {
        double x = i * step;

        std::complex<double> E_total(0.0, 0.0);

        // Ray 0: Direct Path (0 Bounces)
        E_total += std::polar(1.0 / x, -beta * x);

        // Loop over 1-bounce, 2-bounce, and 3-bounce paths
        for (int bounces = 1; bounces <= 3; bounces++) {
            // Using image method: The apparent source moves 2*W further away perpendicularly per bounce
            double image_y = bounces * (2.0 * W);
            double d_n = std::sqrt(std::pow(x, 2) + std::pow(image_y, 2));

            // Incidence angle is relative to the normal of the wall
            double theta_i = std::atan(x / image_y);

            std::complex<double> gamma_m = calcDielectricSlabReflection(theta_i, freq, epsilon_r, wall_thickness);

            // The ray undergoes 'bounces' number of reflections, so we raise gamma_m to that power
            std::complex<double> E_n = std::pow(gamma_m, bounces) * std::polar(1.0 / d_n, -beta * d_n);

            // Multiply by 2 because the corridor is symmetric (Top->Bottom and Bottom->Top sequences)
            E_total += 2.0 * E_n;
        }

        // Convert complex E-field sum to Received Power
        double power_factor = std::norm(E_total);
        double p_rx_dBm = p_tx_dBm + g_tx_dB + g_rx_dB +
                          20.0 * std::log10(lambda / (4.0 * M_PI)) +
                          10.0 * std::log10(power_factor);

        corridorSeries->append(x, p_rx_dBm);

        // Baseline friis
        double p_rx_friis = p_tx_dBm + g_tx_dB + g_rx_dB - 20.0 * std::log10((4.0 * M_PI * x) / lambda);
        friisSeries->append(x, p_rx_friis);
    }

    // 3. Build the Chart
    QChart *chart = new QChart();
    chart->addSeries(corridorSeries);
    chart->addSeries(friisSeries);
    chart->setTitle("Validation: Up to 3-Reflection Urban Canyon with Dielectric Slab modeled Walls (26 GHz)");
    QFont titleFont = chart->titleFont();
    titleFont.setPointSize(16);
    titleFont.setBold(true);
    chart->setTitleFont(titleFont);

    QValueAxis *axisX = new QValueAxis();
    axisX->setTitleText("Distance (m)");
    axisX->setRange(1.0, max_dist);
    axisX->setTickCount(11);
    chart->addAxis(axisX, Qt::AlignBottom);
    friisSeries->attachAxis(axisX);
    corridorSeries->attachAxis(axisX);

    QValueAxis *axisY = new QValueAxis();
    axisY->setTitleText("Received Power (dBm)");
    axisY->setRange(-110.0, -30.0);
    chart->addAxis(axisY, Qt::AlignLeft);
    friisSeries->attachAxis(axisY);
    corridorSeries->attachAxis(axisY);

    // 3. Create the UI Window
    QChartView *chartView = new QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);

    QWidget *window = new QWidget();
    window->setWindowTitle("Validation: Up to 3-Reflection Urban Canyon");
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
        double W = 5.0 * scale;
        double d = 50.0 * scale;

        // Drawn manually

        // Walls (Y = W and Y = -W)
        scene->addLine(0, W, d, W, QPen(Qt::black, 3));
        scene->addLine(0, -W, d, -W, QPen(Qt::black, 3));

        // TX and RX
        scene->addEllipse(-5, -5, 10, 10, QPen(Qt::red), QBrush(Qt::red));
        scene->addEllipse(d-5, -5, 10, 10, QPen(Qt::blue), QBrush(Qt::blue));

        // 0-Bounce (Direct)
        scene->addLine(0, 0, d, 0, QPen(Qt::green, 1, Qt::DashLine));

        // 1-Bounce (Top wall: 1 segment)
        scene->addLine(0, 0, d/2, W, QPen(Qt::magenta, 1));
        scene->addLine(d/2, W, d, 0, QPen(Qt::magenta, 1));

        // 2-Bounce (Top then Bottom: divides X distance into 4 parts)
        scene->addLine(0, 0, d/4, W, QPen(Qt::darkCyan, 1));
        scene->addLine(d/4, W, 3*d/4, -W, QPen(Qt::darkCyan, 1));
        scene->addLine(3*d/4, -W, d, 0, QPen(Qt::darkCyan, 1));

        // 3-Bounce (Top, Bottom, Top: divides X distance into 6 parts)
        scene->addLine(0, 0, d/6, W, QPen(Qt::darkYellow, 1));
        scene->addLine(d/6, W, 3*d/6, -W, QPen(Qt::darkYellow, 1));
        scene->addLine(3*d/6, -W, 5*d/6, W, QPen(Qt::darkYellow, 1));
        scene->addLine(5*d/6, W, d, 0, QPen(Qt::darkYellow, 1));

        scene->setSceneRect(-20, -W - 20, d + 40, 2*W + 40);
        showValidationScene(scene, "Urban Canyon Geometry Snapshot (d=50m), only showing half the rays (symmetric)");
    });

    QObject::connect(btnToggleAxis, &QPushButton::clicked, [max_dist, chart, corridorSeries, friisSeries]() {
        QAbstractAxis *oldAxisX = chart->axes(Qt::Horizontal).constFirst();
        bool isLog = (oldAxisX->type() == QAbstractAxis::AxisTypeLogValue);

        corridorSeries->detachAxis(oldAxisX);
        friisSeries->detachAxis(oldAxisX);

        chart->removeAxis(oldAxisX);
        delete oldAxisX;

        if (isLog) {
            QValueAxis *newAxisX = new QValueAxis();
            newAxisX->setTitleText("Distance (m)");
            newAxisX->setRange(1.0, max_dist);
            newAxisX->setTickCount(11);
            chart->addAxis(newAxisX, Qt::AlignBottom);
            corridorSeries->attachAxis(newAxisX);
            friisSeries->attachAxis(newAxisX);
        } else {
            QLogValueAxis *newAxisX = new QLogValueAxis();
            newAxisX->setTitleText("Distance (m) [Log Scale]");
            newAxisX->setBase(10.0);
            newAxisX->setRange(1.0, max_dist);
            newAxisX->setMinorTickCount(8);
            newAxisX->setLabelFormat("%g");
            chart->addAxis(newAxisX, Qt::AlignBottom);
            corridorSeries->attachAxis(newAxisX);
            friisSeries->attachAxis(newAxisX);
        }
    });

    return window;
}

