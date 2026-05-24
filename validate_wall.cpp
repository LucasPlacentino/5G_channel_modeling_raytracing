#include "validate_wall.h"

#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QVBoxLayout>
#include <cmath>
#include <complex>

QChartView* createTwoRayValidationPlot() {
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
    friisSeries->setName("Friis Free-Space Baseline");
    QPen friisPen(Qt::darkGray);
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
    chart->addSeries(friisSeries);
    chart->addSeries(twoRaySeries);
    chart->setTitle("Validation: Two-Ray Interference Pattern (26 GHz)");

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

    QChartView *chartView = new QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);
    chartView->setMinimumSize(800, 600);
    return chartView;
}

