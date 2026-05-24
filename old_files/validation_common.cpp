#include "validation_common.h"
#include <QGraphicsView>
#include <QPen>
#include <QBrush>
#include <cmath>

// ============================================================================
// GLOBAL GRAPHICS OBJECTS (shared across validation modules)
// ============================================================================

QGraphicsScene* validation_scene = nullptr;
int scene_offset = 30;

// ============================================================================
// SHARED UTILITY FUNCTIONS
// ============================================================================

/**
 * Convert linear power to dBm
 */
qreal powerW_to_dBm(qreal power_W)
{
    if (power_W <= 0) return -999.0;
    return 10.0 * std::log10(power_W * 1000.0);
}

/**
 * Add transmitter graphics to scene
 */
void addTransmitterToScene(QGraphicsScene* scene, Transmitter* transmitter)
{
    if (!scene || !transmitter) return;

    transmitter->graphics->setRect(
        10 * transmitter->x() - 5,
        10 * transmitter->y() - 5,
        10, 10
        );
    QPen tx_pen(Qt::darkGreen);
    tx_pen.setWidthF(2);
    transmitter->graphics->setPen(tx_pen);
    transmitter->graphics->setBrush(QBrush(Qt::green));
    transmitter->graphics->setToolTip(
        QString("Transmitter (%1, %2) - %3 dBm")
            .arg(transmitter->x())
            .arg(transmitter->y())
            .arg(transmitter->getPower_dBm())
        );
    scene->addItem(transmitter->graphics);
}

/**
 * Add receiver graphics to scene
 */
void addReceiverToScene(QGraphicsScene* scene, Receiver* receiver)
{
    if (!scene || !receiver) return;

    receiver->graphics->setRect(
        10 * receiver->x() - 5,
        10 * receiver->y() - 5,
        10, 10
        );
    QPen rx_pen(Qt::darkBlue);
    rx_pen.setWidthF(2);
    receiver->graphics->setPen(rx_pen);
    receiver->graphics->setBrush(QBrush(Qt::blue));
    receiver->graphics->setToolTip(
        QString("Receiver (%1, %2) - Power: %3 W")
            .arg(receiver->x())
            .arg(receiver->y())
            .arg(receiver->power)
        );
    scene->addItem(receiver->graphics);
}

/**
 * Create and display visualization window
 */
QGraphicsView* displayVisualization(QGraphicsScene* scene, int nb_refl)
{
    if (!scene) return nullptr;

    QGraphicsView* view = new QGraphicsView(scene);
    if (nb_refl == 0) {
        view->setWindowTitle("Validation - Free Space - Ray-TracingVisualization");
    } else {
        view->setWindowTitle("Validation - " + QString::number(nb_refl) + " Reflections - Ray-Tracing Visualization");
    }
    view->setGeometry(scene_offset, scene_offset, 1000 + 4*scene_offset, 700 + 4*scene_offset);
    view->show();

    return view;
}
