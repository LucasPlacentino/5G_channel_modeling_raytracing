#ifndef VALIDATION_COMMON_H
#define VALIDATION_COMMON_H

#include <QGraphicsScene>
#include "transmitter.h"
#include "receiver.h"

// ============================================================================
// GLOBAL GRAPHICS OBJECTS (shared across validation modules)
// ============================================================================

extern QGraphicsScene* validation_scene;
extern int scene_offset;

// ============================================================================
// SHARED UTILITY FUNCTIONS
// ============================================================================

/**
 * Convert linear power to dBm
 */
qreal powerW_to_dBm(qreal power_W);

/**
 * Add transmitter graphics to scene
 */
void addTransmitterToScene(QGraphicsScene* scene, Transmitter* transmitter);

/**
 * Add receiver graphics to scene
 */
void addReceiverToScene(QGraphicsScene* scene, Receiver* receiver);

/**
 * Create and display visualization window
 */
QGraphicsView* displayVisualization(QGraphicsScene* scene, int nb_refl);

#endif // VALIDATION_COMMON_H