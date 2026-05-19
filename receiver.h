#ifndef RECEIVER_H
#define RECEIVER_H

#include "ray.h"
#include "transmitter.h"

#include <QGraphicsRectItem>
#include <QVector2D>

#include <QMap> // new

#include <QGraphicsSceneMouseEvent> // Required for mouse click events

class Receiver; // Forward declaration


// Custom graphics item to handle clicks
class ReceiverCellGraphics : public QGraphicsRectItem {
public:
    Receiver* parentReceiver;
    ReceiverCellGraphics(Receiver* parent) : parentReceiver(parent) {}
    
protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
};


class Receiver : public QVector2D // each cell of the simulation grid acts like a receiver
{
public:

    Receiver(qreal x, qreal y, qreal resolution, bool showOutline = false);

    //QGraphicsRectItem* graphics = new QGraphicsRectItem(); // RX's QGraphicsItem
    // It must be initialized in receiver.cpp
    QGraphicsRectItem* graphics; // RX's QGraphicsItem
    
    qreal power; // ! in Watts
    qreal gain;
    qulonglong bitrate_Mbps; // bitrate in Mbps
    qreal Ra = 73; // antenna resistance
    QColor cell_color = QColor(Qt::transparent); // receiver cell color, based on power/bitrate

    // Add this to your public variables in receiver.h
    int connected_tx_selector_index = -1; // new
    Transmitter* connected_tx = nullptr; // new

    QList<Ray*> all_rays; // list of all rays that go to this receiver

    qreal computeTotalPower(Transmitter* transmitter); // returns final total power computation for this RX

    void updateBitrateAndColor(); // update this receiver's bitrate and cell color

    QColor computeColor(qreal value_normalized);

    // new:
    QMap<int, qreal> computeTDL(Transmitter* transmitter);

    ~Receiver(); // destructor to avoid memory leaks
};

#endif // RECEIVER_H
