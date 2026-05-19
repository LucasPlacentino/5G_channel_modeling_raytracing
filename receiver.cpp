#include "receiver.h"

#include <QBrush>
#include <QPen>
#include "parameters.h"

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
        this->cell_color = Qt::black; // Qt::transparent or Qt::black or Qt::darkBlue ?
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

        qDebug() << "--- Power Delay Profile ---";
        for (auto it = RX_TDL_pdp.constBegin(); it != RX_TDL_pdp.constEnd(); ++it) {
            qDebug() << "Tap" << it.key() << "(Delay:" << it.key() * 5 << "ns) : " << it.value() << "dBm";
        }

        event->accept(); // Tell Qt this click was handled
    } else {
        QGraphicsRectItem::mousePressEvent(event); // Pass other clicks (right-click) down
    }
}
