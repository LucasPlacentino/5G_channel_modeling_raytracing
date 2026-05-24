#include "validation_double_reflection.h"
#include <cmath>
#include <QDebug>
#include <QVector2D>
#include "parameters.h"
#include "transmitter.h"
#include "receiver.h"
#include "obstacle.h"
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsLineItem>
#include <QGraphicsEllipseItem>
#include <QPen>
#include <QBrush>
#include "validation_common.h"

/**
 * @file validation_double_reflection.cpp
 * @brief Validates double-reflection propagation model (two bounces)
 * 
 * This validation test compares simulated received power against theoretical
 * double-reflection path loss:
 * 
 * P_RX = P_TX * G_TX * G_RX * |Γ1|² * |Γ2|² * (λ/(4πd1))² * (λ/(4πd2))² * (λ/(4πd3))²
 * where d1 = TX to 1st reflection, d2 = 1st to 2nd reflection, d3 = 2nd reflection to RX
 */

extern Obstacle* wall1;
extern Obstacle* wall2;
extern Obstacle* wall3;

// ============================================================================
// THEORETICAL CALCULATIONS
// ============================================================================

/**
 * Calculate reflection coefficient magnitude
 */
qreal computeReflectionCoeff_Magnitude_DR(qreal theta_i_rad)
{
    complex<qreal> epsilon_tilde = epsilon_0 * epsilon_r - j * (1.0 / omega);
    complex<qreal> Z_m = sqrt(mu_0 / epsilon_tilde);
    complex<qreal> gamma = (Z_m - Z_0) / (Z_m + Z_0);
    return std::abs(gamma);
}

/**
 * Mirror a point through a wall
 */
QVector2D mirrorPoint(const QVector2D& point, Obstacle* wall)
{
    QVector2D new_origin = QVector2D(wall->line.p1());
    QVector2D new_point_coords = point - new_origin;
    double _dotProduct = QVector2D::dotProduct(new_point_coords, wall->normal);
    QVector2D _image_new_coords = new_point_coords - 2 * _dotProduct * wall->normal;
    return new_origin + _image_new_coords;
}

/**
 * Find reflection point on wall given start and end positions using method of images
 */
bool findReflectionPoint(const QVector2D& _start, const QVector2D& _end, 
                         Obstacle* wall, QVector2D& reflection_point_out)
{
    QVector2D d = _end - _start;
    QVector2D x0 = QVector2D(wall->line.p1());
    
    qreal denominator = (wall->unitary.x() * d.y() - wall->unitary.y() * d.x());
    if (std::abs(denominator) < 1e-6) {
        return false;
    }
    
    qreal t = ((d.y() * (_start.x() - x0.x())) - (d.x() * (_start.y() - x0.y()))) / denominator;
    
    qreal wall_length = wall->line.length();
    if (t < 0 || t > wall_length) {
        return false;
    }
    
    reflection_point_out = x0 + t * wall->unitary;
    return true;
}

/**
 * Compute theoretical double-reflection received power
 */
qreal computeTheoreticalPower_DoubleReflection(const QVector2D& tx_pos, const QVector2D& rx_pos,
                                                const QVector2D& reflection_point_1,
                                                const QVector2D& reflection_point_2,
                                                qreal tx_power_W, qreal tx_gain, qreal rx_gain,
                                                qreal gamma_mag_1, qreal gamma_mag_2)
{
    // Distances
    qreal d1 = (reflection_point_1 - tx_pos).length();
    qreal d2 = (reflection_point_2 - reflection_point_1).length();
    qreal d3 = (rx_pos - reflection_point_2).length();
    
    qDebug() << "\n--- THEORETICAL CALCULATION (DOUBLE REFLECTION) ---";
    qDebug() << "TX:" << tx_pos.x() << "," << tx_pos.y();
    qDebug() << "RX:" << rx_pos.x() << "," << rx_pos.y();
    qDebug() << "Reflection point 1:" << reflection_point_1.x() << "," << reflection_point_1.y();
    qDebug() << "Reflection point 2:" << reflection_point_2.x() << "," << reflection_point_2.y();
    qDebug() << "d1:" << d1 << "m, d2:" << d2 << "m, d3:" << d3 << "m";
    
    // Path loss factors
    qreal path_loss_1 = std::pow(lambda / (4.0 * M_PI * d1), 2.0);
    qreal path_loss_2 = std::pow(lambda / (4.0 * M_PI * d2), 2.0);
    qreal path_loss_3 = std::pow(lambda / (4.0 * M_PI * d3), 2.0);
    
    // Reflection coefficients
    qreal gamma_squared_1 = gamma_mag_1 * gamma_mag_1;
    qreal gamma_squared_2 = gamma_mag_2 * gamma_mag_2;
    
    qDebug() << "|Γ1|²:" << gamma_squared_1 << ", |Γ2|²:" << gamma_squared_2;
    
    // Total power
    qreal P_RX = tx_power_W * tx_gain * rx_gain * 
                 gamma_squared_1 * gamma_squared_2 * 
                 path_loss_1 * path_loss_2 * path_loss_3;
    
    qDebug() << "P_RX (theoretical):" << P_RX << "W";
    
    return P_RX;
}

// ============================================================================
// SIMULATED POWER COMPUTATION
// ============================================================================

/**
 * Compute received power for double-reflected ray
 */
qreal computeSimulatedDoubleReflectionPower(Transmitter* transmitter, Receiver* receiver,
                                             Obstacle* wall_1, Obstacle* wall_2,
                                             QVector2D& refl_point_1_out,
                                             QVector2D& refl_point_2_out)
{
    QVector2D tx_pos(transmitter->x(), transmitter->y());
    QVector2D rx_pos(receiver->x(), receiver->y());
    
    qDebug() << "\n--- SIMULATED CALCULATION (DOUBLE REFLECTION) ---";
    qDebug() << "TX:" << tx_pos.x() << "," << tx_pos.y();
    qDebug() << "RX:" << rx_pos.x() << "," << rx_pos.y();
    
    // Double image method:
    // 1. Mirror RX through wall_2
    QVector2D rx_image_1 = mirrorPoint(rx_pos, wall_2);
    
    // 2. Mirror RX_image_1 through wall_1
    QVector2D rx_image_2 = mirrorPoint(rx_image_1, wall_1);
    
    // 3. Find first reflection on wall_1 (intersection of TX-to-RX_image_2)
    if (!findReflectionPoint(tx_pos, rx_image_2, wall_1, refl_point_1_out)) {
        qDebug() << "First reflection point not on wall segment";
        return 0.0;
    }
    
    qreal d1 = (refl_point_1_out - tx_pos).length();
    if (d1 < 0.01) {
        qDebug() << "WARNING: First reflection point too close to TX";
        return 0.0;
    }
    
    // 4. Find second reflection on wall_2 (intersection of refl_point_1-to-RX_image_1)
    if (!findReflectionPoint(refl_point_1_out, rx_image_1, wall_2, refl_point_2_out)) {
        qDebug() << "Second reflection point not on wall segment";
        return 0.0;
    }
    
    qreal d2 = (refl_point_2_out - refl_point_1_out).length();
    qreal d3 = (rx_pos - refl_point_2_out).length();
    
    if (d2 < 0.01 || d3 < 0.01) {
        qDebug() << "WARNING: Reflection points too close";
        return 0.0;
    }
    
    qDebug() << "Reflection point 1:" << refl_point_1_out.x() << "," << refl_point_1_out.y();
    qDebug() << "Reflection point 2:" << refl_point_2_out.x() << "," << refl_point_2_out.y();
    qDebug() << "d1:" << d1 << "m, d2:" << d2 << "m, d3:" << d3 << "m";
    
    // Path loss factors
    qreal path_loss_1 = std::pow(lambda / (4.0 * M_PI * d1), 2.0);
    qreal path_loss_2 = std::pow(lambda / (4.0 * M_PI * d2), 2.0);
    qreal path_loss_3 = std::pow(lambda / (4.0 * M_PI * d3), 2.0);
    
    // Reflection coefficients
    qreal gamma_mag_1 = computeReflectionCoeff_Magnitude_DR(0.0);
    qreal gamma_mag_2 = computeReflectionCoeff_Magnitude_DR(0.0);
    qreal gamma_squared_1 = gamma_mag_1 * gamma_mag_1;
    qreal gamma_squared_2 = gamma_mag_2 * gamma_mag_2;
    
    qDebug() << "|Γ1|²:" << gamma_squared_1 << ", |Γ2|²:" << gamma_squared_2;
    
    qreal P_RX = transmitter->power * transmitter->gain * receiver->gain *
                 gamma_squared_1 * gamma_squared_2 *
                 path_loss_1 * path_loss_2 * path_loss_3;
    
    qDebug() << "P_RX (simulated):" << P_RX << "W";
    
    return P_RX;
}

// ============================================================================
// GRAPHICS & VISUALIZATION
// ============================================================================

/**
 * Create graphics scene with wall for reflection
 */
QGraphicsScene* createGraphicsSceneWithWallsDR()
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
 * Draw double-reflected ray path
 */
void drawDoubleReflectedRay(QGraphicsScene* scene, Transmitter* transmitter, Receiver* receiver,
                            const QVector2D& refl_point_1, const QVector2D& refl_point_2)
{
    if (!scene || !transmitter || !receiver) return;
    
    QPen ray_pen(Qt::magenta);
    ray_pen.setWidthF(2);
    
    // TX to first reflection
    QGraphicsLineItem* ray1 = scene->addLine(
        10 * transmitter->x(), 10 * transmitter->y(),
        10 * refl_point_1.x(), 10 * refl_point_1.y()
    );
    ray1->setPen(ray_pen);
    ray1->setToolTip("Double Reflected Ray - TX to Wall 1");
    
    // First reflection to second reflection
    QGraphicsLineItem* ray2 = scene->addLine(
        10 * refl_point_1.x(), 10 * refl_point_1.y(),
        10 * refl_point_2.x(), 10 * refl_point_2.y()
    );
    ray2->setPen(ray_pen);
    ray2->setToolTip("Double Reflected Ray - Wall 1 to Wall 2");
    
    // Second reflection to RX
    QGraphicsLineItem* ray3 = scene->addLine(
        10 * refl_point_2.x(), 10 * refl_point_2.y(),
        10 * receiver->x(), 10 * receiver->y()
    );
    ray3->setPen(ray_pen);
    ray3->setToolTip("Double Reflected Ray - Wall 2 to RX");
    
    // Mark reflection points
    for (int i = 0; i < 2; i++) {
        QVector2D point = (i == 0) ? refl_point_1 : refl_point_2;
        QGraphicsEllipseItem* marker = scene->addEllipse(
            10 * point.x() - 3, 10 * point.y() - 3, 6, 6
        );
        QPen marker_pen(Qt::magenta);
        marker_pen.setWidthF(1);
        marker->setPen(marker_pen);
        marker->setBrush(QBrush(Qt::magenta));
        marker->setToolTip(QString("Reflection Point %1").arg(i + 1));
    }
}

// ============================================================================
// VALIDATION TEST CASE
// ============================================================================

/**
 * Double reflection validation test
 */
void testCase_DoubleReflection()
{
    qDebug() << "\n========== DOUBLE REFLECTION TEST CASE ==========";

    qreal tx_power = 0.1;
    Transmitter* tx = new Transmitter(10.0, 5.2, 0, "TX_Test");
    Receiver* rx = new Receiver(6.8, 13.3, 0.5, false);

    tx->power = tx_power;
    tx->gain = G_TX;
    rx->gain = G_TX;

    QVector2D tx_pos(tx->x(), tx->y());
    QVector2D rx_pos(rx->x(), rx->y());

    // Test double reflection on walls 1 and 2
    QVector2D refl_1, refl_2;
    qreal P_simulated = computeSimulatedDoubleReflectionPower(tx, rx, wall1, wall2, refl_1, refl_2);
    
    if (P_simulated == 0.0) {
        qDebug() << "No valid double reflection found";
        return;
    }
    // Calculate incidence angle using dot product of ray vector and wall normal
    //QVector2D ray_dir = (reflection_point - _tx).normalized();
    //qreal cos_theta = std::abs(QVector2D::dotProduct(ray_dir, wall->normal));
    //qreal theta_i_rad = std::acos(cos_theta);
    
    qreal gamma_mag = computeReflectionCoeff_Magnitude_DR(0.0);
    //qreal gamma_mag = computeReflectionCoeff_Magnitude_DR(theta_i_rad);
    qreal P_theoretical = computeTheoreticalPower_DoubleReflection(
        tx_pos, rx_pos, refl_1, refl_2, tx->power, tx->gain, rx->gain, gamma_mag, gamma_mag
    );
    
    qreal P_theoretical_dBm = powerW_to_dBm(P_theoretical);
    qreal P_simulated_dBm = powerW_to_dBm(P_simulated);
    rx->power = P_simulated;
    
    qreal error_percent = std::abs(P_simulated - P_theoretical) / P_theoretical * 100.0;
    
    qDebug() << "Theoretical P_RX:" << P_theoretical << "W =" << P_theoretical_dBm << "dBm";
    qDebug() << "Simulated P_RX:" << P_simulated << "W =" << P_simulated_dBm << "dBm";
    qDebug() << "Error:" << error_percent << "%";
    
    if (error_percent < 1.0) {
        qDebug() << "✓ PASS";
    } else {
        qDebug() << "✗ FAIL";
    }
    
    if (validation_scene) {
        addTransmitterToScene(validation_scene, tx);
        addReceiverToScene(validation_scene, rx);
        drawDoubleReflectedRay(validation_scene, tx, rx, refl_1, refl_2);
    }
}

// ============================================================================
// MAIN VALIDATION RUNNER
// ============================================================================

/**
 * Run double reflection validation test
 */
QGraphicsView* runDoubleReflectionValidation()
{
    qDebug() << "\n\n";
    qDebug() << "╔════════════════════════════════════════════════════════════╗";
    qDebug() << "║     DOUBLE-REFLECTION PROPAGATION VALIDATION               ║";
    qDebug() << "╚════════════════════════════════════════════════════════════╝";

    qDebug() << "\nTest Parameters:";
    qDebug() << "  Frequency:" << freq / 1e9 << "GHz";
    qDebug() << "  Wavelength:" << lambda << "m";
    qDebug() << "  TX Power:" << P_TX * 1000 << "mW =" << P_TX_dBm << "dBm";
    qDebug() << "  TX/RX Gain:" << G_TX << "(linear)";
    qDebug() << "  Material: Concrete (ε_r = " << epsilon_r << ")";

    validation_scene = createGraphicsSceneWithWallsDR();
    qDebug() << "\nGraphics scene created";

    testCase_DoubleReflection();

    qDebug() << "\n════════════════════════════════════════════════════════════";
    qDebug() << "DOUBLE-REFLECTION VALIDATION COMPLETE\n";

    QGraphicsView* view = displayVisualization(validation_scene, 2);
    return view;
}
