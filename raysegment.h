#ifndef RAYSEGMENT_H
#define RAYSEGMENT_H

#include <QGraphicsLineItem>
#include <QLineF>


class RaySegment : public QLineF
{
public:
    qreal distance; //? not used ?

    //QGraphicsLineItem* graphics = new QGraphicsLineItem(); // segment's QGraphicsItem // removed from here to reduce memory

    RaySegment(qreal start_x, qreal start_y, qreal end_x, qreal end_y);

    //~RaySegment(); // destructor to avoid memory leaks
};

#endif // RAYSEGMENT_H