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
 * 
 * This validation test compares simulated received power against theoretical
 * free-space path loss computed using the Friis transmission equation:
 * 
 * P_RX = P_TX * G_TX * G_RX * (lambda / (4*pi*d))^2
 */

// ============================================================================
// GLOBAL GRAPHICS OBJECTS
// ============================================================================

QGraphicsScene* validation_scene = nullptr;

// ============================================================================
// THEORETICAL CALCULATIONS
// ============================================================================

/**
 * Compute theoretical free-space received power using Friis transmission equation
 * @param distance_m Distance in meters between TX and RX
 * @param tx_power_W Transmitter power in Watts
 * @param tx_gain Transmitter antenna gain (linear)
 * @param rx_gain Receiver antenna gain (linear)
 * @return Received power in Watts
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
 * @param distance_m Distance in meters
 * @return Path loss in dB (positive value)
 */
qreal computePathLoss_dB(qreal distance_m)
{
    // PL(dB) = 20*log10(4*pi*d/lambda)
    qreal PL = 20.0 * std::log10((4.0 * M_PI * distance_m) / lambda);
    return PL;
}

/**
 * Convert linear power to dBm
 */
qreal powerW_to_dBm(qreal power_W)
{
    if (power_W <= 0) return -999.0;
    return 10.0 * std::log10(power_W * 1000.0);
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
 * Uses the formula: P_RX = |H(f)|^2 * P_TX
 * where |H(f)|^2 includes path loss and antenna gains
 */
qreal computeSimulatedDirectRayPower(Transmitter* transmitter, Receiver* receiver)
{
    // Extract positions from objects
    QVector2D tx_pos(transmitter->x(), transmitter->y());
    QVector2D rx_pos(receiver->x(), receiver->y());
    
    // Distance
    qreal d = (rx_pos - tx_pos).length();
    
    // Free-space path loss: (lambda / (4*pi*d))^2
    qreal path_loss_linear = std::pow(lambda / (4.0 * M_PI * d), 2.0);
    
    // Received power using antenna formula with object's properties
    qreal P_RX = (60.0 * lambda * lambda) / (8.0 * M_PI * M_PI * receiver->Ra) 
                 * transmitter->gain * receiver->Ra / receiver->Ra  // using receiver's gain implicitly in its resistance
                 * transmitter->power * path_loss_linear;
    
    return P_RX;
}

// ============================================================================
// GRAPHICS & VISUALIZATION
// ============================================================================

/**
 * Create graphics scene with two perpendicular walls
 */
QGraphicsScene* createGraphicsSceneWithWalls()
{
    QGraphicsScene* scene = new QGraphicsScene();
    scene->setSceneRect(0, 0, 200, 200);

    // Create and add two perpendicular walls
    // Wall 1: Horizontal wall
    Obstacle* wall1 = new Obstacle(QVector2D(0, 5), QVector2D(16, 5), GenericWall, 0.4);
    scene->addItem(wall1->graphics);
    qDebug() << "Wall 1 created (horizontal)";

    // Wall 2: Vertical wall
    Obstacle* wall2 = new Obstacle(QVector2D(7, 0), QVector2D(7, 12), GenericWall, 0.4);
    scene->addItem(wall2->graphics);
    qDebug() << "Wall 2 created (vertical)";

    return scene;
}

/**
 * Draw transmitter to scene
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

/**
 * Create and display visualization window
 */
void displayVisualization(QGraphicsScene* scene)
{
    if (!scene) return;

    QGraphicsView* view = new QGraphicsView(scene);
    view->setWindowTitle("Free-Space Validation - Ray Visualization");
    view->setGeometry(100, 100, 800, 800);
    view->show();
}


// ============================================================================
// VALIDATION TEST CASES
// ============================================================================

/**
 * Test 1: Simple 1-meter distance
 */
void testCase_1meter()
{
    qDebug() << "\n========== TEST CASE 1: 1 METER DISTANCE ==========";
    
    qreal distance = 1.0; // 1 meter

    qreal tx_power = 0.1; // 100 mW = 20 dBm
    qreal tx_gain = G_TX;
    qreal rx_gain = G_TX; // Assume dipole with same gain at RX
    qreal antenna_R = 73.0; // Receiver antenna resistance

    // Create TX and RX objects
    Transmitter* tx = new Transmitter(2.0, 2.0, 0, "TX_Test1");
    Receiver* rx = new Receiver(2.0 + distance, 2.0, 0.5, false);

    // Set TX/RX parameters
    tx->power = tx_power;
    tx->gain = tx_gain;
    rx->Ra = antenna_R;
    
    // Theoretical calculation
    qreal P_theoretical = computeTheoreticalPower_FriisEquation(distance, tx->power, tx->gain, rx->gain);
    qreal PL_dB = computePathLoss_dB(distance);
    qreal P_theoretical_dBm = powerW_to_dBm(P_theoretical);
    
    // Simulated calculation
    // QVector2D tx_pos(0.0, 0.0);
    // QVector2D rx_pos(distance, 0.0);
    // qreal P_simulated = computeSimulatedDirectRayPower(tx_pos, rx_pos, tx_power, tx_gain, rx_gain, antenna_R);
    qreal P_simulated = computeSimulatedDirectRayPower(tx, rx);
    rx->power = P_simulated;  // Store in receiver object
    qreal P_simulated_dBm = powerW_to_dBm(P_simulated);
    
    // Error analysis
    qreal error_percent = std::abs(P_simulated - P_theoretical) / P_theoretical * 100.0;
    
    qDebug() << "TX Position:" << tx->x() << "," << tx->y();
    qDebug() << "RX Position:" << rx->x() << "," << rx->y();
    qDebug() << "Distance:" << distance << "m";
    qDebug() << "TX Power:" << tx->power << "W =" << tx->getPower_dBm() << "dBm";
    qDebug() << "TX Gain:" << tx->gain;
    qDebug() << "RX Antenna Resistance:" << rx->Ra << "Ohm";
    qDebug() << "Theoretical P_RX:" << P_theoretical << "W =" << P_theoretical_dBm << "dBm";
    qDebug() << "Simulated P_RX:" << P_simulated << "W =" << P_simulated_dBm << "dBm";
    qDebug() << "Path Loss:" << PL_dB << "dB";
    qDebug() << "Error:" << error_percent << "%";
    
    if (error_percent < 1.0) {
        qDebug() << "✓ PASS: Error within tolerance";
    } else {
        qDebug() << "✗ FAIL: Error exceeds 1% tolerance";
    }

    // // Add to scene
    // if (validation_scene) {
    //     addTransmitterToScene(validation_scene, tx);
    //     addReceiverToScene(validation_scene, rx);
    //     drawDirectRay(validation_scene, tx, rx);
    // }
}

/**
 * Test 2: Realistic distance (10 meters)
 */
void testCase_10meters()
{
    qDebug() << "\n========== TEST CASE 2: 10 METER DISTANCE ==========";
    
    qreal distance = 10.0; // 10 meters

    qreal tx_power = 0.1; // 100 mW = 20 dBm
    qreal tx_gain = G_TX;
    qreal rx_gain = G_TX;
    qreal antenna_R = 73.0;

    // Create TX and RX objects
    Transmitter* tx = new Transmitter(1.0, 1.0, 1, "TX_Test2");
    Receiver* rx = new Receiver(1.0 + distance, 1.0, 0.5, false);

    // Set TX/RX parameters
    tx->power = tx_power;
    tx->gain = tx_gain;
    rx->Ra = antenna_R;

    // Theoretical
    qreal P_theoretical = computeTheoreticalPower_FriisEquation(
        distance, tx->power, tx->gain, rx->gain
    );
    qreal PL_dB = computePathLoss_dB(distance);
    qreal P_theoretical_dBm = powerW_to_dBm(P_theoretical);
    
    // Simulated
    // QVector2D tx_pos(0.0, 0.0);
    // QVector2D rx_pos(distance, 0.0);
    // qreal P_simulated = computeSimulatedDirectRayPower(tx_pos, rx_pos, tx_power, tx_gain, rx_gain, antenna_R);
    qreal P_simulated = computeSimulatedDirectRayPower(tx, rx);
    rx->power = P_simulated;
    qreal P_simulated_dBm = powerW_to_dBm(P_simulated);
    
    // Error
    qreal error_percent = std::abs(P_simulated - P_theoretical) / P_theoretical * 100.0;
    
    qDebug() << "TX Position:" << tx->x() << "," << tx->y();
    qDebug() << "RX Position:" << rx->x() << "," << rx->y();
    qDebug() << "Distance:" << distance << "m";
    qDebug() << "TX Power:" << tx->power << "W =" << tx->getPower_dBm() << "dBm";
    qDebug() << "TX Gain:" << tx->gain;
    qDebug() << "RX Antenna Resistance:" << rx->Ra << "Ohm";
    qDebug() << "Theoretical P_RX:" << P_theoretical << "W =" << P_theoretical_dBm << "dBm";
    qDebug() << "Simulated P_RX:" << P_simulated << "W =" << P_simulated_dBm << "dBm";
    qDebug() << "Path Loss:" << PL_dB << "dB";
    qDebug() << "Error:" << error_percent << "%";
    
    if (error_percent < 1.0) {
        qDebug() << "✓ PASS: Error within tolerance";
    } else {
        qDebug() << "✗ FAIL: Error exceeds 1% tolerance";
    }

    // Add to scene
    if (validation_scene) {
        addTransmitterToScene(validation_scene, tx);
        addReceiverToScene(validation_scene, rx);
        drawDirectRay(validation_scene, tx, rx);
    }
}

/**
 * Test 3: Long distance (100 meters) - typical BS coverage
 */
void testCase_100meters()
{
    qDebug() << "\n========== TEST CASE 3: 100 METER DISTANCE ==========";
    
    qreal distance = 100.0; // 100 meters

    qreal tx_power = 0.1;
    qreal tx_gain = G_TX;
    qreal rx_gain = G_TX;
    qreal antenna_R = 73.0;

    // Create TX and RX objects
    Transmitter* tx = new Transmitter(0.0, 0.0, 2, "TX_Test3");
    Receiver* rx = new Receiver(distance, 0.0, 0.5, false);

    // Set TX/RX parameters
    tx->power = tx_power;
    tx->gain = tx_gain;
    rx->Ra = antenna_R;

    // Theoretical
    qreal P_theoretical = computeTheoreticalPower_FriisEquation(
        distance, tx->power, tx->gain, rx->gain
    );
    qreal PL_dB = computePathLoss_dB(distance);
    qreal P_theoretical_dBm = powerW_to_dBm(P_theoretical);
    
    // Simulated
    // QVector2D tx_pos(0.0, 0.0);
    // QVector2D rx_pos(distance, 0.0);
    // qreal P_simulated = computeSimulatedDirectRayPower(tx_pos, rx_pos, tx_power, tx_gain, rx_gain, antenna_R);
    qreal P_simulated = computeSimulatedDirectRayPower(tx, rx);
    rx->power = P_simulated;
    qreal P_simulated_dBm = powerW_to_dBm(P_simulated);
    
    // Error
    qreal error_percent = std::abs(P_simulated - P_theoretical) / P_theoretical * 100.0;
    
    qDebug() << "TX Position:" << tx->x() << "," << tx->y();
    qDebug() << "RX Position:" << rx->x() << "," << rx->y();
    qDebug() << "Distance:" << distance << "m";
    qDebug() << "TX Power:" << tx->power << "W =" << tx->getPower_dBm() << "dBm";
    qDebug() << "TX Gain:" << tx->gain;
    qDebug() << "RX Antenna Resistance:" << rx->Ra << "Ohm";
    qDebug() << "Theoretical P_RX:" << P_theoretical << "W =" << P_theoretical_dBm << "dBm";
    qDebug() << "Simulated P_RX:" << P_simulated << "W =" << P_simulated_dBm << "dBm";
    qDebug() << "Path Loss:" << PL_dB << "dB";
    qDebug() << "Error:" << error_percent << "%";
    
    if (error_percent < 1.0) {
        qDebug() << "✓ PASS: Error within tolerance";
    } else {
        qDebug() << "✗ FAIL: Error exceeds 1% tolerance";
    }

    // // Add to scene
    // if (validation_scene) {
    //     addTransmitterToScene(validation_scene, tx);
    //     addReceiverToScene(validation_scene, rx);
    //     drawDirectRay(validation_scene, tx, rx);
    // }
}


// ============================================================================
// MAIN VALIDATION RUNNER
// ============================================================================

/**
 * Run all free-space validation tests
 */
void runFreeSpaceValidation()
{
    qDebug() << "\n\n";
    qDebug() << "╔════════════════════════════════════════════════════════════╗";
    qDebug() << "║     FREE-SPACE PROPAGATION VALIDATION (PHASE 1 - TEST 1)   ║";
    qDebug() << "╚════════════════════════════════════════════════════════════╝";
    
    qDebug() << "\nTest Parameters:";
    qDebug() << "  Frequency:" << freq / 1e9 << "GHz";
    qDebug() << "  Wavelength:" << lambda << "m";
    qDebug() << "  TX Power:" << P_TX * 1000 << "mW =" << P_TX_dBm << "dBm";
    qDebug() << "  TX Gain:" << G_TX << "(linear)";
    qDebug() << "  Antenna Resistance:" << 73.0 << "Ohm";
    qDebug() << "  Vacuum Impedance (Z0):" << Z_0 << "Ohm";
    
    // Create graphics scene with two perpendicular walls
    validation_scene = createGraphicsSceneWithWalls();
    qDebug() << "\nGraphics scene created with 2 perpendicular obstacle walls";
    

    // Run all tests
    testCase_1meter();
    testCase_10meters();
    testCase_100meters();
    
    qDebug() << "\n════════════════════════════════════════════════════════════";
    qDebug() << "FREE-SPACE VALIDATION COMPLETE\n";

    // Display visualization
    qDebug() << "\nDisplaying visualization window...";
    displayVisualization(validation_scene);
}
