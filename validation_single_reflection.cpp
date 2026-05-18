#include "validation_single_reflection.h"
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
#include "validation_common.h"

/**
 * @file validation_single_reflection.cpp
 * @brief Validates single-reflection propagation model (no direct path, one bounce only)
 * 
 * This validation test compares simulated received power against theoretical
 * single-reflection path loss:
 * 
 * P_RX = P_TX * G_TX * G_RX * |Γ|² * (λ/(4πd1))² * (λ/(4πd2))²
 * where d1 = TX to reflection point, d2 = reflection point to RX
 */

Obstacle* wall1;
Obstacle* wall2;
Obstacle* wall3;

// ============================================================================
// THEORETICAL CALCULATIONS
// ============================================================================

/**
 * Calculate reflection coefficient magnitude for perpendicular incidence (normal incidence)
 * Assumes plane wave reflection at interface
 */
qreal computeReflectionCoeff_Magnitude(qreal theta_i_rad)
{
    // For normal incidence (theta_i = 0), perpendicular polarization
    // |Γ| = |Z_material - Z_0| / |Z_material + Z_0|
    
    // For concrete: Z_m ≈ complex impedance of material
    // Using simplified formula for normal incidence
    complex<qreal> epsilon_tilde = epsilon_0 * epsilon_r - j * (1.0 / omega); // material properties
    complex<qreal> Z_m = sqrt(mu_0 / epsilon_tilde);
    
    complex<qreal> gamma = (Z_m - Z_0) / (Z_m + Z_0);
    return std::abs(gamma);
}

/**
 * Calculate reflection point on wall given TX and RX positions using method of images
 * Returns reflection point and validates it's on the wall segment
 */
bool calculateReflectionPoint_MethodOfImages(const QVector2D& _tx, const QVector2D& _rx, 
                                              Obstacle* wall, QVector2D& reflection_point_out)
{
    // Method of images: 
    // 1. Compute virtual image of RX through the wall
    // 2. Find intersection of TX-to-RX_image line with the wall
    
    // Compute image of RX
    QVector2D new_origin = QVector2D(wall->line.p1());
    QVector2D new_point_coords = _rx - new_origin;
    double _dotProduct = QVector2D::dotProduct(new_point_coords, wall->normal);
    QVector2D _image_new_coords = new_point_coords - 2 * _dotProduct * wall->normal;
    QVector2D rx_image = new_origin + _image_new_coords;
    
    // Find intersection between TX-to-RX_image line and wall
    QVector2D d = rx_image - _tx;
    QVector2D x0 = QVector2D(wall->line.p1());
    
    qreal denominator = (wall->unitary.x() * d.y() - wall->unitary.y() * d.x());
    if (std::abs(denominator) < 1e-6) {
        return false; // Lines are parallel
    }
    
    qreal t = ((d.y() * (_tx.x() - x0.x())) - (d.x() * (_tx.y() - x0.y()))) / denominator;
    
    // Check if intersection is on the wall segment (0 <= t <= wall_length)
    qreal wall_length = wall->line.length();
    if (t < 0 || t > wall_length) {
        qDebug() << "Intersection point outside wall segment: t =" << t << ", wall_length =" << wall_length;
        return false;
    }
    
    reflection_point_out = x0 + t * wall->unitary;
    return true;
}

/**
 * Compute theoretical single-reflection received power
 */
qreal computeTheoreticalPower_SingleReflection(const QVector2D& tx_pos, const QVector2D& rx_pos,
                                                const QVector2D& reflection_point,
                                                qreal tx_power_W, qreal tx_gain, qreal rx_gain,
                                                qreal reflection_coeff_mag)
{
    // Distances
    qreal d1 = (reflection_point - tx_pos).length();
    qreal d2 = (rx_pos - reflection_point).length();
    
    qDebug() << "\n--- THEORETICAL CALCULATION ---";
    qDebug() << "TX:" << tx_pos.x() << "," << tx_pos.y();
    qDebug() << "RX:" << rx_pos.x() << "," << rx_pos.y();
    qDebug() << "Reflection point:" << reflection_point.x() << "," << reflection_point.y();
    qDebug() << "d1 (TX to reflection):" << d1 << "m";
    qDebug() << "d2 (reflection to RX):" << d2 << "m";
    
    // Path loss factors
    qreal path_loss_1 = std::pow(lambda / (4.0 * M_PI * d1), 2.0);
    qreal path_loss_2 = std::pow(lambda / (4.0 * M_PI * d2), 2.0);
    
    qDebug() << "λ:" << lambda << "m";
    qDebug() << "λ/(4πd1):" << (lambda / (4.0 * M_PI * d1));
    qDebug() << "path_loss_1 = (λ/(4πd1))²:" << path_loss_1;
    qDebug() << "λ/(4πd2):" << (lambda / (4.0 * M_PI * d2));
    qDebug() << "path_loss_2 = (λ/(4πd2))²:" << path_loss_2;
    
    // Reflection coefficient power
    qreal gamma_squared = reflection_coeff_mag * reflection_coeff_mag;
    
    qDebug() << "|Γ|:" << reflection_coeff_mag;
    qDebug() << "|Γ|²:" << gamma_squared;
    
    qDebug() << "P_TX:" << tx_power_W << "W";
    qDebug() << "G_TX:" << tx_gain;
    qDebug() << "G_RX:" << rx_gain;
    
    // Total power: P_RX = P_TX * G_TX * G_RX * |Γ|² * (λ/(4πd1))² * (λ/(4πd2))²
    qreal P_RX = tx_power_W * tx_gain * rx_gain * gamma_squared * path_loss_1 * path_loss_2;
    
    qDebug() << "P_RX = P_TX × G_TX × G_RX × |Γ|² × path_loss_1 × path_loss_2";
    qDebug() << "P_RX = " << tx_power_W << "×" << tx_gain << "×" << rx_gain << "×" << gamma_squared 
             << "×" << path_loss_1 << "×" << path_loss_2;
    qDebug() << "P_RX (theoretical):" << P_RX << "W";
    
    return P_RX;
}


// ============================================================================
// SIMULATED POWER COMPUTATION
// ============================================================================

/**
 * Compute received power for single-reflected ray
 */
qreal computeSimulatedSingleReflectionPower(Transmitter* transmitter, Receiver* receiver, 
                                             Obstacle* reflection_wall)
{
    QVector2D tx_pos(transmitter->x(), transmitter->y());
    QVector2D rx_pos(receiver->x(), receiver->y());
    
    qDebug() << "\n--- SIMULATED CALCULATION ---";
    qDebug() << "TX:" << tx_pos.x() << "," << tx_pos.y();
    qDebug() << "RX:" << rx_pos.x() << "," << rx_pos.y();
    
    // Find reflection point using method of images
    QVector2D reflection_point;
    if (!calculateReflectionPoint_MethodOfImages(tx_pos, rx_pos, reflection_wall, reflection_point)) {
        qDebug() << "Reflection point not on wall segment";
        return 0.0;
    }
    
    qDebug() << "Reflection point:" << reflection_point.x() << "," << reflection_point.y();
    
    // Distances
    qreal d1 = (reflection_point - tx_pos).length();
    qreal d2 = (rx_pos - reflection_point).length();
    
    qDebug() << "d1 (TX to reflection):" << d1 << "m";
    qDebug() << "d2 (reflection to RX):" << d2 << "m";
    
    if (d1 < 0.01 || d2 < 0.01) {
        qDebug() << "WARNING: Reflection point too close to TX or RX";
        return 0.0;
    }
    
    // Path loss factors
    qreal path_loss_1 = std::pow(lambda / (4.0 * M_PI * d1), 2.0);
    qreal path_loss_2 = std::pow(lambda / (4.0 * M_PI * d2), 2.0);
    
    qDebug() << "λ:" << lambda << "m";
    qDebug() << "path_loss_1 = (λ/(4πd1))²:" << path_loss_1;
    qDebug() << "path_loss_2 = (λ/(4πd2))²:" << path_loss_2;
    
    // Reflection coefficient
    qreal gamma_mag = computeReflectionCoeff_Magnitude(0.0);
    qreal gamma_squared = gamma_mag * gamma_mag;
    
    qDebug() << "|Γ|:" << gamma_mag;
    qDebug() << "|Γ|²:" << gamma_squared;
    
    qDebug() << "P_TX:" << transmitter->power << "W";
    qDebug() << "G_TX:" << transmitter->gain;
    qDebug() << "G_RX:" << receiver->gain;
    
    qreal P_RX = transmitter->power * transmitter->gain * receiver->gain * 
                 gamma_squared * path_loss_1 * path_loss_2;
    
    qDebug() << "P_RX = P_TX × G_TX × G_RX × |Γ|² × path_loss_1 × path_loss_2";
    qDebug() << "P_RX = " << transmitter->power << "×" << transmitter->gain << "×" << receiver->gain 
             << "×" << gamma_squared << "×" << path_loss_1 << "×" << path_loss_2;
    qDebug() << "P_RX (simulated):" << P_RX << "W";
    
    return P_RX;
}

// ============================================================================
// GRAPHICS & VISUALIZATION
// ============================================================================


/**
 * Create graphics scene with wall for reflection
 */
QGraphicsScene* createGraphicsSceneWithWallsSR()
{
    QGraphicsScene* scene = new QGraphicsScene();
    scene->setSceneRect(-scene_offset, -scene_offset, 1000 + scene_offset*2, 700 + scene_offset*2);
    QBrush bgBrush(Qt::black);
    scene->setBackgroundBrush(bgBrush);

    // Wall 1: Horizontal wall
    wall1 = new Obstacle(QVector2D(0, 0), QVector2D(50, 0), GenericWall, 0.4);
    scene->addItem(wall1->graphics);
    qDebug() << "Wall 1 created (horizontal)";

    // Wall 2: Vertical wall
    wall2 = new Obstacle(QVector2D(0, 0), QVector2D(0, 70), GenericWall, 0.4);
    scene->addItem(wall2->graphics);
    qDebug() << "Wall 2 created (vertical)";

    // Wall 3: Slanted wall
    wall3 = new Obstacle(QVector2D(50, 0), QVector2D(100, 20), GenericWall, 0.4);
    scene->addItem(wall3->graphics);
    qDebug() << "Wall 3 created (slanted)";

    return scene;
}

/**
 * Check if two points are on the same side of a wall
 */
bool checkSameSideOfWall(const QVector2D& _normal, const QVector2D& _TX, const QVector2D& _RX)
{
    // Returns true if _TX and _RX are on the same side of the wall (using the wall's normal vector)
    bool res = (QVector2D::dotProduct(_normal, _RX) > 0 == QVector2D::dotProduct(_normal, _TX) > 0);
    return res;
}

/**
 * Draw reflected ray path (TX -> reflection point -> RX)
 */
void drawReflectedRay(QGraphicsScene* scene, Transmitter* transmitter, Receiver* receiver, 
                      const QVector2D& reflection_point)
{
    if (!scene || !transmitter || !receiver) return;
    
    // TX to reflection point
    QGraphicsLineItem* ray1 = scene->addLine(
        10 * transmitter->x(), 
        10 * transmitter->y(),
        10 * reflection_point.x(), 
        10 * reflection_point.y()
    );
    QPen ray_pen(Qt::yellow);
    ray_pen.setWidthF(2);
    ray1->setPen(ray_pen);
    ray1->setToolTip("Reflected Ray - TX to Wall");
    
    // Reflection point to RX
    QGraphicsLineItem* ray2 = scene->addLine(
        10 * reflection_point.x(), 
        10 * reflection_point.y(),
        10 * receiver->x(), 
        10 * receiver->y()
    );
    ray2->setPen(ray_pen);
    ray2->setToolTip("Reflected Ray - Wall to RX");
    
    // Mark reflection point
    QGraphicsEllipseItem* reflection_marker = scene->addEllipse(
        10 * reflection_point.x() - 3,
        10 * reflection_point.y() - 3,
        6, 6
    );
    QPen marker_pen(Qt::cyan);
    marker_pen.setWidthF(1);
    reflection_marker->setPen(marker_pen);
    reflection_marker->setBrush(QBrush(Qt::cyan));
    reflection_marker->setToolTip("Reflection Point");
}

// ============================================================================
// VALIDATION TEST CASE
// ============================================================================

/**
 * Single reflection validation test
 */
void testCase_SingleReflection()
{
    qDebug() << "\n========== SINGLE REFLECTION TEST CASE ==========";

    qreal tx_power = 0.1; // 100 mW = 20 dBm
    qreal tx_gain = G_TX;
    qreal rx_gain = G_TX;

    // Create TX and RX objects
    Transmitter* tx = new Transmitter(10.0, 5.2, 0, "TX_Test");
    Receiver* rx = new Receiver(6.8, 13.3, 0.5, false);

    // Set TX/RX parameters
    tx->power = tx_power;
    tx->gain = tx_gain;
    rx->gain = G_TX;

    QVector2D tx_pos(tx->x(), tx->y());
    QVector2D rx_pos(rx->x(), rx->y());

    // Store results for all walls
    QList<Obstacle*> walls = {wall1, wall2, wall3};
    QList<QString> wall_names = {"Horizontal", "Vertical", "Slanted"};

    // Reflection coefficient (same for all walls in this test)
    qreal gamma_mag = computeReflectionCoeff_Magnitude(0.0);

    for (int i = 0; i < walls.length(); i++) {
        Obstacle* wall = walls[i];
        QString wall_name = wall_names[i];

        qDebug() << "\n--- Wall " << i+1 << ":" << wall_name << " ---";

        // Check if TX and RX are on the same side of the wall
        if (!checkSameSideOfWall(wall->normal, tx_pos, rx_pos)) {
            qDebug() << "  TX and RX not on same side - no reflection";
            continue;
        }

        QVector2D reflection_point; // input/output of below
        if (!calculateReflectionPoint_MethodOfImages(tx_pos, rx_pos, wall, reflection_point)) {
            qDebug() << "  Reflection point not on wall segment";
            continue;
        }

        qreal d1 = (reflection_point - tx_pos).length();
        qreal d2 = (rx_pos - reflection_point).length();

        // Validate distances
        if (d1 < 0.01 || d2 < 0.01) {
            qDebug() << "  WARNING: Reflection point too close to TX or RX";
            continue;
        }

        // Check if reflection point lies within wall segment bounds
        qreal wall_length = wall->line.length();
        qreal dist_to_p1 = (reflection_point - QVector2D(wall->line.p1())).length();
        if (dist_to_p1 > wall_length) {
            qDebug() << "  WARNING: Reflection point is outside wall segment";
            continue;
        }

        qDebug() << "  Reflection Point:" << reflection_point.x() << "," << reflection_point.y();
        qDebug() << "  Distance TX->Reflection:" << d1 << "m";
        qDebug() << "  Distance Reflection->RX:" << d2 << "m";

        // Theoretical
        qreal P_theoretical = computeTheoreticalPower_SingleReflection(
            tx_pos, rx_pos, reflection_point, tx->power, tx->gain, rx->gain, gamma_mag
        );
        qreal P_theoretical_dBm = powerW_to_dBm(P_theoretical);

        // Simulated (recalculate for this wall)
        qreal P_simulated = computeSimulatedSingleReflectionPower(tx, rx, wall);
        qreal P_simulated_dBm = powerW_to_dBm(P_simulated);

        // Error
        qreal error_percent = std::abs(P_simulated - P_theoretical) / P_theoretical * 100.0;

        qDebug() << "  Theoretical P_RX:" << P_theoretical << "W =" << P_theoretical_dBm << "dBm";
        qDebug() << "  Simulated P_RX:" << P_simulated << "W =" << P_simulated_dBm << "dBm";
        qDebug() << "  Error:" << error_percent << "%";

        if (error_percent < 1.0) {
            qDebug() << "  ✓ PASS";
        } else {
            qDebug() << "  ✗ FAIL";
        }

        // Draw reflection
        if (validation_scene) {
            drawReflectedRay(validation_scene, tx, rx, reflection_point);
        }
    }

    // Add TX and RX to scene once
    if (validation_scene) {
        addTransmitterToScene(validation_scene, tx);
        addReceiverToScene(validation_scene, rx);
    }
}

// ============================================================================
// MAIN VALIDATION RUNNER
// ============================================================================

/**
 * Run single reflection validation test
 */
QGraphicsView* runSingleReflectionValidation()
{
    qDebug() << "\n\n";
    qDebug() << "╔════════════════════════════════════════════════════════════╗";
    qDebug() << "║      SINGLE-REFLECTION PROPAGATION VALIDATION              ║";
    qDebug() << "╚════════════════════════════════════════════════════════════╝";

    qDebug() << "\nTest Parameters:";
    qDebug() << "  Frequency:" << freq / 1e9 << "GHz";
    qDebug() << "  Wavelength:" << lambda << "m";
    qDebug() << "  TX Power:" << P_TX * 1000 << "mW =" << P_TX_dBm << "dBm";
    qDebug() << "  TX/RX Gain:" << G_TX << "(linear)";
    qDebug() << "  Material: Concrete (ε_r = " << epsilon_r << ")";

    // Create graphics scene with reflection wall
    validation_scene = createGraphicsSceneWithWallsSR();
    qDebug() << "\nGraphics scene created with reflection wall";

    // Run test
    testCase_SingleReflection();

    qDebug() << "\n════════════════════════════════════════════════════════════";
    qDebug() << "SINGLE-REFLECTION VALIDATION COMPLETE\n";

    // Display visualization
    qDebug() << "\nDisplaying visualization window...";
    QGraphicsView* view = displayVisualization(validation_scene);

    return view;
}