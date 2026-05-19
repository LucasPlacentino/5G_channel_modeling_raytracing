#include "receiver.h"

#include <QBrush>
#include <QPen>
#include "parameters.h"

// new: (for TDL charts)
#include <QtCharts/QChartView>

//#include <QtCharts/QBarSeries>
//#include <QtCharts/QBarSet>
//#include <QtCharts/QBarCategoryAxis>

#include <QtCharts/QScatterSeries>
#include <QtCharts/QLineSeries>
#include <QtCharts/QLegendMarker>
#include <QtCharts/QCategoryAxis>
#include <QFont>

#include <QShortcut>
#include <QKeySequence>
#include <QFileDialog>
#include <QPixmap>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPdfWriter>
#include <QPainter>
#include <QPageSize>

//#include <QtCharts/QValueAxis>
#include <cmath>
void showPDPChartWindow(const QMap<int, qreal>& pdp_dBm);

/*
 Sensitivity | Bit rate
     -80 dBm | 100 Mb/s (lower: comms impossible)
     -60 dBm | 1 Gb/s  (higher: data rate capped)
*/

// Conversion to bitrate (beware log scale)
static const qreal max_sensitivity_mW = std::pow(10.0, max_sensitivity_dBm / 10.0);
static const qreal min_sensitivity_mW = std::pow(10.0, min_sensitivity_dBm / 10.0);
static const qreal max_bitrate_dB = 10*log10(max_bitrate_Mbps);
static const qreal min_bitrate_dB = 10*log10(min_bitrate_Mbps);

Receiver::Receiver(qreal x, qreal y, qreal resolution, bool showOutline) {
    // Receiver object constructor
    QBrush rxBrush(Qt::black);
    QPen rxPen;
    if (showOutline){
        rxPen.setColor(Qt::black);
        rxPen.setWidthF(10*0.01);
    } else {
        rxPen.setColor(Qt::transparent);
        rxPen.setWidth(0);
    }

    // Initialize the custom graphics item and pass 'this' receiver to it
    this->graphics = new ReceiverCellGraphics(this);

    this->setX(x);
    this->setY(y);
    this->graphics->setToolTip(QString("Receiver x=%1 y=%2").arg(this->x(),this->y()));
    this->graphics->setBrush(rxBrush);
    this->graphics->setPen(rxPen);
    this->graphics->setRect(10*(x-resolution/2),10*(y-resolution/2),10*resolution,10*resolution);
    this->graphics->setAcceptHoverEvents(true);
}

// Receiver::~Receiver() {
//     // Receiver destructor to avoid memory leaks
//     qDeleteAll(this->all_rays);
//     // Only delete the graphics if it is NOT managed by a scene
//     if (this->graphics && this->graphics->scene() == nullptr) {
//         delete this->graphics;
//     }
// }

void Receiver::updateBitrateAndColor()
{
    qreal bitrate;
    qreal power_dBm = 10*std::log10(this->power*1000);
    if (this->power != this->power) {
        bitrate = 9999999999999999.9;
        this->cell_color = QColor::fromRgb(255,192,203); // pink
        qDebug() << "--- ! ERROR: Cell has NaN power ! ---";
    } else if (power_dBm > max_sensitivity_dBm) {
        bitrate = max_bitrate_Mbps; //in Mbps, 40 Gbps max from -40 dBm
        this->cell_color = QColor::fromRgb(255,0,0); // red
    } else if (power_dBm < min_sensitivity_dBm) { // 50 Mbps at -90 dBm
        bitrate = 0; //in Mbps, no connection (0 Mbps)
        this->cell_color = Qt::transparent; // Qt::transparent or Qt::black or Qt::darkBlue ?
    } else {

        //bitrate = min_bitrate_Mbps + (((this->power - min_power_mW/1000) / (max_power_mW/1000 - min_power_mW/1000)) * (max_bitrate_Mbps - min_bitrate_Mbps));
        // OR ?
        //global max_bitrate_dB etc above


        qreal bitrate_dB = min_bitrate_dB + (((power_dBm - min_sensitivity_dBm) / (max_sensitivity_dBm - min_sensitivity_dBm)) * (max_bitrate_dB - min_bitrate_dB));
        //qDebug() << "bitrate dB:" << bitrate_dB;
        bitrate = pow(10.0, bitrate_dB / 10.0); // back to linear
        //qDebug() << "bitrate (Mbps):" << bitrate;

        //qreal value_normalized = (power_dBm - min_power_dBm) / (max_power_dBm - min_power_dBm);
        // or
        //qreal value_normalized = (this->power*1000 - min_power_mW) / (max_power_mW - min_power_mW);
        // or
        //qreal value_normalized = (qreal(bitrate) - qreal(min_bitrate_Mbps)) / (qreal(max_bitrate_Mbps) - qreal(min_bitrate_Mbps));
        // or
        qreal value_normalized = (bitrate_dB - min_bitrate_dB) / (max_bitrate_dB - min_bitrate_dB);
        //qDebug() << "value_normalized:" << value_normalized;

        QColor color = computeColor(value_normalized);
        // testing in monochrome :
        //QColor color = QColor::fromRgbF(value_normalized,value_normalized,value_normalized);
        //qDebug() << "cell color:" << color;

        this->cell_color = color;
    }
    //// DEBUG:
    ////this->cell_color = Qt::black; // set ALL cells to black

    this->bitrate_Mbps = bitrate;
}

QColor Receiver::computeColor(qreal value)
{
    // Maps a normalized value to a 5 color gradient

    // Define colors for the heatmap gradient
    QColor colors[] = {
        QColor(0, 0, 255),   // Blue
        QColor(0, 255, 255), // Cyan
        QColor(0, 255, 0),   // Green
        QColor(255, 255, 0), // Yellow
        QColor(255, 0, 0)    // Red
    };

    // Determine which two colors to interpolate between
    int index1 = value * 4;
    int index2 = index1 + 1;

    // Calculate interpolation factor
    qreal factor = (value * 4) - index1;

    // Interpolate between the two colors
    QColor color = colors[index1].toRgb();
    QColor next_color = colors[index2].toRgb();

    int red = color.red() + factor * (next_color.red() - color.red());
    int green = color.green() + factor * (next_color.green() - color.green());
    int blue = color.blue() + factor * (next_color.blue() - color.blue());

    return QColor(red, green, blue);
}


qreal Receiver::computeTotalPower(Transmitter* transmitter) // returns final total power computation for this RX

{
    // TODO: far-field cut-off protection ?
    qreal distance_to_tx = this->distanceToPoint(*transmitter);
    if (distance_to_tx < far_field_min_distance) {
        return 0.0; // Return 0 power; model is invalid here
    }

    // Initial E-field amplitude at 1m
    // for THIS transmitter
    qreal E0 = sqrt(60.0 * transmitter->gain * transmitter->power);

    complex<qreal> E_tot(0, 0);

    // Get the coordinates of this specific transmitter
    QPointF tx_pos(transmitter->x(), transmitter->y());

    for (Ray* ray : std::as_const(this->all_rays)) {

        // FILTER: Only add E-fields of rays that belong to THIS transmitter.
        if (ray->tx_selector_index != transmitter->selector_index) {
            continue;
        }

        complex<qreal> ray_coeff(1, 0);
        for (complex<qreal> coeff : std::as_const(ray->coeffsList)) {
            ray_coeff *= coeff; // Product of all Gamma_m and T_m for this ray
        }

        qreal d = ray->getTotalDistance();

        // Coherent addition: E_n = E0 * (Coeffs) * exp(-j*beta*d) / d
        complex<qreal> E_ray = E0 * ray_coeff * exp(-j * beta_0 * d) / d;
        E_tot += E_ray;
    }

    qreal h_e = lambda / M_PI; // Effective height for lambda/2 dipole

    // P_RX = (1 / 8*Ra) * |h_e * E_tot|^2
    qreal res = (1.0 / (8.0 * Ra)) * pow(abs(h_e * E_tot), 2);

    if (res != res) qDebug() << "computeTotalPower: NaN !!!";

    return res;
}


// new:
QMap<int, qreal> Receiver::computeTDL(Transmitter* transmitter)
{
    QMap<int, complex<qreal>> complex_taps;
    qreal E0 = sqrt(60.0 * transmitter->gain * transmitter->power);
    qreal tap_distance_resolution = double(c) / B; // 1.5 meters

    // 1. Bin rays into taps
    for (Ray* ray : std::as_const(this->all_rays)) {
        if (ray->tx_selector_index != transmitter->selector_index) {
            continue;
        }

        qreal d = ray->getTotalDistance();

        // Find which tap this ray falls into based on its distance
        int tap_index = qFloor(d/tap_distance_resolution);

        complex<qreal> ray_coeff(1, 0);
        for (complex<qreal> coeff : std::as_const(ray->coeffsList)) {
            ray_coeff *= coeff;
        }

        complex<qreal> E_ray = E0 * ray_coeff * exp(-j * beta_0 * d) / d;

        // Add E-field coherently to the existing tap
        complex_taps[tap_index] += E_ray;
    }

    // 2. Compute power for each tap
    QMap<int, qreal> power_delay_profile;

    for (auto it = complex_taps.constBegin(); it != complex_taps.constEnd(); ++it) {
        int tap = it.key();
        complex<qreal> E_tot_tap = it.value();

        qreal tap_power = (1.0 / (8.0 * R_a)) * pow(abs(effective_height * E_tot_tap), 2);

        // Store as dBm for plotting
        power_delay_profile[tap] = 10.0 * std::log10(tap_power * 1000.0);

    }

    return power_delay_profile; // Key: Tap Index, Value: Power in dBm
}

void ReceiverCellGraphics::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        qDebug() << "Clicked cell at" << parentReceiver->x() << "," << parentReceiver->y();

        Transmitter* tx = parentReceiver->connected_tx;
        if (tx == nullptr) {
            qDebug() << "Cell has no connected base station (Dead zone).";
            event->accept();
            return;
        }

        // TODO:
        // TRIGGER TDL LOGIC HERE:
        // ...
        QMap<int, qreal> RX_TDL_pdp = parentReceiver->computeTDL(tx);

        showPDPChartWindow(RX_TDL_pdp);

        qDebug() << "--- Power Delay Profile ---";
        for (auto it = RX_TDL_pdp.constBegin(); it != RX_TDL_pdp.constEnd(); ++it) {
            qDebug() << "Tap" << it.key() << "(Delay:" << it.key() * 5 << "ns) : " << it.value() << "dBm";
        }

        event->accept(); // Tell Qt this click was handled
    } else {
        QGraphicsRectItem::mousePressEvent(event); // Pass other clicks (right-click) down
    }
}


// new (for TDL charts):
void showPDPChartWindow(const QMap<int, qreal>& pdp_dBm) 
{
    if (pdp_dBm.isEmpty()) return;

    // QBarSeries *series = new QBarSeries();
    // QBarSet *powerSet = new QBarSet("Received Power");
    // QStringList categories;

    // int max_tap = pdp_dBm.lastKey();
    ////qreal tap_duration_ns = 5.0; // 1 / 200MHz = 5 ns
    qreal tap_duration_ns = 1 / (B/1e9);
    
    // Variables for RMS Delay Spread
    qreal sum_P = 0.0;
    qreal sum_P_tau = 0.0;
    qreal sum_P_tau2 = 0.0;
    qreal max_power_dBm = -999.0;

    qreal max_delay_ns = 0.0; // ?

    // Series 1: The Dots at the peaks
    QScatterSeries *scatterSeries = new QScatterSeries();
    scatterSeries->setName("Multipath Components");
    scatterSeries->setMarkerShape(QScatterSeries::MarkerShapeCircle);
    scatterSeries->setMarkerSize(8.0);
    scatterSeries->setColor(Qt::blue);

    // Series 2: The vertical lines (stems) dropping to the floor
    QLineSeries *stemSeries = new QLineSeries();
    QPen stemPen(Qt::blue);
    stemPen.setWidth(2);
    stemSeries->setPen(stemPen);
    stemSeries->setName("Stems");

    // NEW: Create the custom Category Axis here
    QCategoryAxis *axisX = new QCategoryAxis();
    axisX->setTitleText("Delay (ns)");
    QFont axisFont;
    axisFont.setPointSize(12); // Larger than default, smaller than main title
    axisX->setTitleFont(axisFont);
    axisX->setLabelsPosition(QCategoryAxis::AxisLabelsPositionOnValue); // Put the label exactly on the line
    // Add this line to rotate the labels vertically (read from bottom to top)
    axisX->setLabelsAngle(-90);

    // // 1. Process data for the chart and the physics metrics
    // for (int i = 0; i <= max_tap; i++) {
    //     qreal delay_ns = i * tap_duration_ns;
    //     categories << QString::number(delay_ns);

    //     if (pdp_dBm.contains(i)) {
    //         qreal pwr_dBm = pdp_dBm[i];
    //         *powerSet << pwr_dBm;
            
    //         if (pwr_dBm > max_power_dBm) max_power_dBm = pwr_dBm;

    //         // Convert back to linear mW for RMS math
    //         qreal P_linear = qPow(10.0, pwr_dBm / 10.0);
    //         sum_P += P_linear;
    //         sum_P_tau += P_linear * delay_ns;
    //         sum_P_tau2 += P_linear * qPow(delay_ns, 2);

    //     } else {
    //         *powerSet << -120.0; // Noise floor for empty taps
    //     }
    // }

    // 1. Process ONLY the taps that actually exist in the map
    for (auto it = pdp_dBm.constBegin(); it != pdp_dBm.constEnd(); ++it) {
        int tap = it.key();
        qreal pwr_dBm = it.value();
        qreal delay_ns = tap * tap_duration_ns;

        if (pwr_dBm > max_power_dBm) max_power_dBm = pwr_dBm;
        if (delay_ns > max_delay_ns) max_delay_ns = delay_ns;

        // Convert back to linear mW for RMS math
        qreal P_linear = std::pow(10.0, pwr_dBm / 10.0); 
        sum_P += P_linear;
        sum_P_tau += P_linear * delay_ns;
        sum_P_tau2 += P_linear * std::pow(delay_ns, 2);

        // Add peak dot to chart
        scatterSeries->append(delay_ns, pwr_dBm);

        // Draw the vertical stem down to the noise floor (-120 dBm)
        stemSeries->append(delay_ns, -120.0);
        stemSeries->append(delay_ns, pwr_dBm);
        stemSeries->append(delay_ns, -120.0); // Return to floor so it doesn't draw a diagonal line to the next peak

        // NEW: Add a specific marker/label for this exact tap
        axisX->append(QString::number(delay_ns), delay_ns);
    }

    // 2. Compute RMS Delay Spread
    qreal mean_tau = sum_P_tau / sum_P;
    qreal rms_delay_spread = qSqrt((sum_P_tau2 / sum_P) - qPow(mean_tau, 2));

    // // 3. Build the Chart
    // series->append(powerSet);
    // QChart *chart = new QChart();
    // chart->addSeries(series);

    // 3. Build the Chart
    QChart *chart = new QChart();
    chart->addSeries(stemSeries); // Add stem first so it draws behind the dots
    chart->addSeries(scatterSeries);
    // Hide the stem legend marker to keep the UI clean
    const auto markers = chart->legend()->markers(stemSeries);
    for (QLegendMarker *marker : markers) {
        marker->setVisible(false);
    }
    
    // Title with Advanced Metrics
    QString title = QString("Power Delay Profile (RMS Delay Spread: %1 ns)").arg(rms_delay_spread, 0, 'f', 2);
    chart->setTitle(title);
    // Increase Chart Title size
    QFont titleFont = chart->titleFont();
    titleFont.setPointSize(16); // Adjust size here (e.g., 14, 16, 18)
    titleFont.setBold(true);
    chart->setTitleFont(titleFont);
    chart->setAnimationOptions(QChart::SeriesAnimations);

    // // X-Axis (Delay in ns)
    // QBarCategoryAxis *axisX = new QBarCategoryAxis();
    // axisX->append(categories);
    // axisX->setTitleText("Delay (ns)");
    // chart->addAxis(axisX, Qt::AlignBottom);
    // series->attachAxis(axisX);

    // X-Axis (Delay in ns) - Now a continuous ValueAxis
    // QValueAxis *axisX = new QValueAxis();
    // axisX->setTitleText("Delay (ns)");
    // axisX->setRange(0, max_delay_ns + 20.0); // Add a 20ns padding on the right
    // axisX->setTickCount(10); // Automatically space out the labels nicely
    // chart->addAxis(axisX, Qt::AlignBottom);
    // stemSeries->attachAxis(axisX);
    // scatterSeries->attachAxis(axisX);
    // X-Axis (Delay in ns) - Now using the CategoryAxis we built in the loop
    axisX->setRange(0, max_delay_ns + 20.0);
    chart->addAxis(axisX, Qt::AlignBottom);
    stemSeries->attachAxis(axisX);
    scatterSeries->attachAxis(axisX);

    // // Y-Axis (Power in dBm)
    // QValueAxis *axisY = new QValueAxis();
    // axisY->setRange(-120.0, max_power_dBm + 5.0); // Dynamic range
    // axisY->setTitleText("Power (dBm)");
    // chart->addAxis(axisY, Qt::AlignLeft);
    // series->attachAxis(axisY);

    // Y-Axis (Power in dBm)
    QValueAxis *axisY = new QValueAxis();
    axisY->setTitleText("Power (dBm)");
    axisY->setTitleFont(axisFont);
    axisY->setRange(-120.0, max_power_dBm + 5.0); // Dynamic range based on strongest signal
    chart->addAxis(axisY, Qt::AlignLeft);
    stemSeries->attachAxis(axisY);
    scatterSeries->attachAxis(axisY);

    // // 4. Create and Show the Floating Window
    // QChartView *chartView = new QChartView(chart);
    // chartView->setRenderHint(QPainter::Antialiasing);
    // chartView->setWindowTitle("TDL Impulse Response");
    // chartView->resize(800, 400);
    //
    // // Crucial: delete the window from memory when the user closes it
    // chartView->setAttribute(Qt::WA_DeleteOnClose);

    // 4. Create the Chart View
    QChartView *chartView = new QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);

    // 5. Create a Wrapper Window with a Layout
    QWidget *window = new QWidget();
    window->setWindowTitle("TDL Impulse Response");
    window->resize(850, 500);
    window->setAttribute(Qt::WA_DeleteOnClose);

    QVBoxLayout *mainLayout = new QVBoxLayout(window);
    mainLayout->addWidget(chartView, 1);

    // 6. Create the Export Buttons
    QHBoxLayout *buttonLayout = new QHBoxLayout();
    QPushButton *btnSavePng = new QPushButton("Save as PNG");
    //QPushButton *btnSavePdf = new QPushButton("Save as PDF");

    btnSavePng->setMinimumHeight(30);
    //btnSavePdf->setMinimumHeight(30);

    buttonLayout->addWidget(btnSavePng);
    //buttonLayout->addWidget(btnSavePdf);
    buttonLayout->addStretch();

    mainLayout->addLayout(buttonLayout);

    // 7. Connect Button Clicks to Save Functions
    QObject::connect(btnSavePng, &QPushButton::clicked, [window, chartView]() {
        QString fileName = QFileDialog::getSaveFileName(window, "Save Chart as Image", "", "PNG Image (*.png)");
        if (!fileName.isEmpty()) {
            QPixmap pixmap = chartView->grab();
            pixmap.save(fileName);
        }
    });

    // QObject::connect(btnSavePdf, &QPushButton::clicked, [window, chartView]() {
    //     QString fileName = QFileDialog::getSaveFileName(window, "Save Chart as PDF", "", "PDF Document (*.pdf)");
    //     if (!fileName.isEmpty()) {
    //         QPdfWriter writer(fileName);
    //         writer.setCreator("5G Channel Ray Tracer");
    //         writer.setPageSize(QPageSize(QPageSize::A4));
    //         writer.setPageOrientation(QPageLayout::Landscape);
    //
    //         QPainter painter(&writer);
    //         chartView->render(&painter);
    //         painter.end();
    //     }
    // });

    // --- Add Keyboard Shortcut (Ctrl+S) ---
    QShortcut *saveShortcut = new QShortcut(QKeySequence::Save, window); // Automatically maps to Ctrl+S

    QObject::connect(saveShortcut, &QShortcut::activated, [window, chartView]() {
        QString fileName = QFileDialog::getSaveFileName(
            window,
            "Save Chart as Image",
            QDir::currentPath(),
            "PNG Image (*.png);;JPEG (*.jpg)"
            );

        if (!fileName.isEmpty()) {
            bool success = chartView->grab().save(fileName);
            if (success) {
                qDebug() << "Chart successfully saved to:" << fileName;
            }
        }
    });
    
    // Show the window independently of the main thread blocking
    //chartView->show();
    window->show();
}
