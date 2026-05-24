#include "validation_free_space.h"
#include <cmath>
#include <iostream>
#include <iomanip>
#include <QDebug>
#include <QVector2D>
#include "parameters.h"
#include "ray.h"
#include "raysegment.h"
#include "transmitter.h"
#include "receiver.h"
#include "obstacle.h"
#include "validation_common.h"
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsLineItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsRectItem>
#include <QPen>
#include <QBrush>
#include <QApplication>

/**
 * @file validation_free_space.cpp
 * @brief Validates the free-space propagation model (direct path only, no obstacles)
 */

// ============================================================================
// THEORETICAL CALCULATIONS
// ============================================================================

/**
 * Compute theoretical free-space received power using Friis transmission equation
 */
qreal computeTheoreticalPower_FriisEquation(qreal distance_m, qreal tx_power_W, 
                                             qreal tx_gain, qreal rx_gain)
{
    qreal path_loss_factor = (lambda / (4.0 * M_PI * distance_m));
    qreal P_RX = tx_power_W * tx_gain * rx_gain * path_loss_factor * path_loss_factor;
    return P_RX;
}

/**
 * Compute theoretical path loss in dB
 */
qreal computePathLoss_dB(qreal distance_m)
{
    qreal PL = 20.0 * std::log10((4.0 * M_PI * distance_m) / lambda);
    return PL;
}

/**
 * Convert dBm to linear power in Watts
 */
qreal powerdBm_to_W(qreal power_dBm)
{
    return std::pow(10.0, power_dBm / 10.0) / 1000.0;
}

// ============================================================================
// SIMULATED POWER COMPUTATION (using project classes)
// ============================================================================

/**
 * Compute received power for a single direct ray
 */
qreal computeSimulatedDirectRayPower(Transmitter* transmitter, Receiver* receiver)
{
    QVector2D tx_pos(transmitter->x(), transmitter->y());
    QVector2D rx_pos(receiver->x(), receiver->y());
    
    qreal d = (rx_pos - tx_pos).length();
    qreal path_loss_linear = std::pow(lambda / (4.0 * M_PI * d), 2.0);
    qreal P_RX = transmitter->power * transmitter->gain * receiver->gain * path_loss_linear;
    
    return P_RX;
}

// ============================================================================
// GRAPHICS & VISUALIZATION
// ============================================================================

/**
 * Create graphics scene (free space)
 */
QGraphicsScene* createGraphicsSceneFS()
{
    QGraphicsScene* scene = new QGraphicsScene();
    scene->setSceneRect(-scene_offset, -scene_offset, 1000 + scene_offset*2, 700 + scene_offset*2);
    QBrush bgBrush(Qt::black);
    scene->setBackgroundBrush(bgBrush);

    return scene;
}

/**
 * Draw direct ray path between transmitter and receiver
 */
void drawDirectRay(QGraphicsScene* scene, Transmitter* transmitter, Receiver* receiver)
{
    if (!scene || !transmitter || !receiver) return;
    
    QGraphicsLineItem* ray_graphics = scene->addLine(
        10 * transmitter->x(), 
        10 * transmitter->y(),
        10 * receiver->x(), 
        10 * receiver->y()
    );
    QPen ray_pen(Qt::red);
    ray_pen.setWidthF(2);
    ray_graphics->setPen(ray_pen);
    ray_graphics->setToolTip("Direct Ray Path");
}

// ============================================================================
// VALIDATION TEST CASE
// ============================================================================

void testCase()
{
    qDebug() << "\n========== TEST CASE ==========";

    qreal tx_power = 0.1;
    qreal tx_gain = G_TX;
    qreal rx_gain = G_TX;
    qreal antenna_R = 73.0;

    Transmitter* tx = new Transmitter(10.0, 5.2, 0, "TX_Test");
    Receiver* rx = new Receiver(6.8, 13.3, 0.5, false);

    QPointF tx_coord = tx->getCoordinates();
    QPointF rx_coord = rx->toPointF();

    qreal distance = std::sqrt(std::pow(rx_coord.x() - tx_coord.x(), 2) + std::pow(rx_coord.y() - tx_coord.y(), 2));

    tx->power = tx_power;
    tx->gain = tx_gain;
    rx->Ra = antenna_R;
    rx->gain = G_RX;

    qreal P_theoretical = computeTheoreticalPower_FriisEquation(distance, tx->power, tx->gain, rx->gain);
    qreal PL_dB = computePathLoss_dB(distance);
    qreal P_theoretical_dBm = powerW_to_dBm(P_theoretical);
    
    qreal P_simulated = computeSimulatedDirectRayPower(tx, rx);
    rx->power = P_simulated;
    qreal P_simulated_dBm = powerW_to_dBm(P_simulated);
    
    qreal error_percent = std::abs(P_simulated - P_theoretical) / P_theoretical * 100.0;
    
    qDebug() << "TX Position:" << tx->x() << "," << tx->y();
    qDebug() << "RX Position:" << rx->x() << "," << rx->y();
    qDebug() << "Distance:" << distance << "m";
    qDebug() << "TX Power:" << tx->power << "W =" << tx->getPower_dBm() << "dBm";
    qDebug() << "TX Gain:" << tx->gain << "RX Gain:" << rx->gain;
    qDebug() << "RX Antenna Resistance:" << rx->Ra << "Ohm";
    qDebug() << "Theoretical P_RX:" << P_theoretical << "W =" << P_theoretical_dBm << "dBm";
    qDebug() << "Simulated P_RX:" << P_simulated << "W =" << P_simulated_dBm << "dBm";
    qDebug() << "Path Loss:" << PL_dB << "dB";
    qDebug() << "Error:" << error_percent << "%";
    
    if (error_percent < 1.0) {
        qDebug() << "✓ PASS: Error within tolerance (<1%)";
    } else {
        qDebug() << "✗ FAIL: Error exceeds 1% tolerance";
    }

    if (validation_scene) {
        addTransmitterToScene(validation_scene, tx);
        addReceiverToScene(validation_scene, rx);
        drawDirectRay(validation_scene, tx, rx);
    }
}

// ============================================================================
// MAIN VALIDATION RUNNER
// ============================================================================

QGraphicsView* runFreeSpaceValidation()
{
    qDebug() << "\n\n";
    qDebug() << "╔════════════════════════════════════════════════════════════╗";
    qDebug() << "║     FREE-SPACE PROPAGATION VALIDATION (PHASE 1 - TEST 1)   ║";
    qDebug() << "╚════════════════════════════════════════════════════════════╝";
    
    qDebug() << "\nTest Parameters:";
    qDebug() << "  Frequency:" << freq / 1e9 << "GHz";
    qDebug() << "  Wavelength:" << lambda << "m";
    qDebug() << "  TX Power:" << P_TX * 1000 << "mW =" << P_TX_dBm << "dBm";
    qDebug() << "  TX Gain:" << G_TX << "RX Gain:" << G_RX << "(linear)";
    qDebug() << "  Antenna Resistance:" << 73.0 << "Ohm";
    qDebug() << "  Vacuum Impedance (Z0):" << Z_0 << "Ohm";
    
    validation_scene = createGraphicsSceneFS();
    qDebug() << "\nGraphics scene created";
    
    testCase();
    
    qDebug() << "\n════════════════════════════════════════════════════════════";
    qDebug() << "FREE-SPACE VALIDATION COMPLETE\n";

    qDebug() << "\nDisplaying visualization window...";
    QGraphicsView* view = displayVisualization(validation_scene, 0);

    return view;
}