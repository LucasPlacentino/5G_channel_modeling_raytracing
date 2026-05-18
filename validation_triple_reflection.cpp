#include "validation_triple_reflection.h"
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
 * @file validation_triple_reflection.cpp
 * @brief Validates triple-reflection propagation model (three bounces)
 * 
 * This validation test compares simulated received power against theoretical
 * triple-reflection path loss:
 * 
 * P_RX = P_TX * G_TX * G_RX * |Γ1|² * |Γ2|² * |Γ3|² * (λ/(4πd1))² * ... * (λ/(4πd4))²
 * where d1...d4 are the four path segments
 */

extern Obstacle* wall1;
extern Obstacle* wall2;
extern Obstacle* wall3;
Obstacle* wall4;

// ============================================================================
// THEORETICAL CALCULATIONS
// ============================================================================

/**
 * Calculate reflection coefficient magnitude
 */
qreal computeReflectionCoeff_Magnitude_TR(qreal theta_i_rad)
{
    complex<qreal> epsilon_tilde = epsilon_0 * epsilon_r - j * (1.0 / omega);
    complex<qreal> Z_m = sqrt(mu_0 / epsilon_tilde);
    complex<qreal> gamma = (Z_m - Z_0) / (Z_m + Z_0);
    return std::abs(gamma);
}

/**
 * Mirror a point through a wall
 */
QVector2D mirrorPoint_TR(const QVector2D& point, Obstacle* wall)
{
    QVector2D new_origin = QVector2D(wall->line.p1());
    QVector2D new_point_coords = point - new_origin;
    double _dotProduct = QVector2D::dotProduct(new_point_coords, wall->normal);
    QVector2D _image_new_coords = new_point_coords - 2 * _dotProduct * wall->normal;
    return new_origin + _image_new_coords;
}

/**
 * Find reflection point on wall given start and end positions
 */
bool findReflectionPoint_TR(const QVector2D& _start, const QVector2D& _end, 
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
 * Compute theoretical triple-reflection received power
 */
qreal computeTheoreticalPower_TripleReflection(const QVector2D& tx_pos, const QVector2D& rx_pos,
                                                const QVector2D& refl_1, const QVector2D& refl_2,
                                                const QVector2D& refl_3,
                                                qreal tx_power_W, qreal tx_gain, qreal rx_gain,
                                                qreal gamma_mag_1, qreal gamma_mag_2, qreal gamma_mag_3)
{
    // Distances
    qreal d1 = (refl_1 - tx_pos).length();
    qreal d2 = (refl_2 - refl_1).length();
    qreal d3 = (refl_3 - refl_2).length();
    qreal d4 = (rx_pos - refl_3).length();
    
    qDebug() << "\n--- THEORETICAL CALCULATION (TRIPLE REFLECTION) ---";
    qDebug() << "TX:" << tx_pos.x() << "," << tx_pos.y();
    qDebug() << "RX:" << rx_pos.x() << "," << rx_pos.y();
    qDebug() << "d1:" << d1 << "m, d2:" << d2 << "m, d3:" << d3 << "m, d4:" << d4 << "m";
    
    // Path loss factors
    qreal path_loss_1 = std::pow(lambda / (4.0 * M_PI * d1), 2.0);
    qreal path_loss_2 = std::pow(lambda / (4.0 * M_PI * d2), 2.0);
    qreal path_loss_3 = std::pow(lambda / (4.0 * M_PI * d3), 2.0);
    qreal path_loss_4 = std::pow(lambda / (4.0 * M_PI * d4), 2.0);
    
    // Reflection coefficients
    qreal gamma_sq_1 = gamma_mag_1 * gamma_mag_1;
    qreal gamma_sq_2 = gamma_mag_2 * gamma_mag_2;
    qreal gamma_sq_3 = gamma_mag_3 * gamma_mag_3;
    
    qDebug() << "|Γ1|²:" << gamma_sq_1 << ", |Γ2|²:" << gamma_sq_2 << ", |Γ3|²:" << gamma_sq_3;
    
    // Total power
    qreal P_RX = tx_power_W * tx_gain * rx_gain * 
                 gamma_sq_1 * gamma_sq_2 * gamma_sq_3 * 
                 path_loss_1 * path_loss_2 * path_loss_3 * path_loss_4;
    
    qDebug() << "P_RX (theoretical):" << P_RX << "W";
    
    return P_RX;
}

// ============================================================================
// SIMULATED POWER COMPUTATION
// ============================================================================

/**
 * Helper: Try a specific reflection path (ordered wall combination)
 */
qreal tryReflectionPath(Transmitter* transmitter, Receiver* receiver,
                        Obstacle* wall_1, Obstacle* wall_2, Obstacle* wall_3,
                        QVector2D& refl_1_out, QVector2D& refl_2_out, QVector2D& refl_3_out)
{
    QVector2D tx_pos(transmitter->x(), transmitter->y());
    QVector2D rx_pos(receiver->x(), receiver->y());
    
    // Apply method of images: mirror in REVERSE order of reflections
    QVector2D rx_image_1 = mirrorPoint_TR(rx_pos, wall_3);
    QVector2D rx_image_2 = mirrorPoint_TR(rx_image_1, wall_2);
    QVector2D rx_image_3 = mirrorPoint_TR(rx_image_2, wall_1);
    
    // Find first reflection on wall_1 (TX to rx_image_3)
    if (!findReflectionPoint_TR(tx_pos, rx_image_3, wall_1, refl_1_out)) {
        return 0.0;
    }
    
    qreal d1 = (refl_1_out - tx_pos).length();
    if (d1 < 0.01) return 0.0;
    
    // Find second reflection on wall_2 (refl_1 to rx_image_2)
    if (!findReflectionPoint_TR(refl_1_out, rx_image_2, wall_2, refl_2_out)) {
        return 0.0;
    }
    
    qreal d2 = (refl_2_out - refl_1_out).length();
    if (d2 < 0.01) return 0.0;
    
    // Find third reflection on wall_3 (refl_2 to rx_image_1)
    if (!findReflectionPoint_TR(refl_2_out, rx_image_1, wall_3, refl_3_out)) {
        return 0.0;
    }
    
    qreal d3 = (refl_3_out - refl_2_out).length();
    qreal d4 = (rx_pos - refl_3_out).length();
    
    if (d3 < 0.01 || d4 < 0.01) {
        return 0.0;
    }
    
    // Compute power
    qreal path_loss_1 = std::pow(lambda / (4.0 * M_PI * d1), 2.0);
    qreal path_loss_2 = std::pow(lambda / (4.0 * M_PI * d2), 2.0);
    qreal path_loss_3 = std::pow(lambda / (4.0 * M_PI * d3), 2.0);
    qreal path_loss_4 = std::pow(lambda / (4.0 * M_PI * d4), 2.0);
    
    qreal gamma_mag_1 = computeReflectionCoeff_Magnitude_TR(0.0);
    qreal gamma_mag_2 = computeReflectionCoeff_Magnitude_TR(0.0);
    qreal gamma_mag_3 = computeReflectionCoeff_Magnitude_TR(0.0);
    qreal gamma_sq_1 = gamma_mag_1 * gamma_mag_1;
    qreal gamma_sq_2 = gamma_mag_2 * gamma_mag_2;
    qreal gamma_sq_3 = gamma_mag_3 * gamma_mag_3;
    
    qreal P_RX = transmitter->power * transmitter->gain * receiver->gain *
                 gamma_sq_1 * gamma_sq_2 * gamma_sq_3 *
                 path_loss_1 * path_loss_2 * path_loss_3 * path_loss_4;
    
    return P_RX;
}

/**
 * Recursive: try all combinations of walls for reflections
 */
void tryAllReflectionCombinations(Transmitter* transmitter, Receiver* receiver,
                                  QList<Obstacle*> available_walls,
                                  QList<Obstacle*> walls_used,
                                  QVector2D& best_refl_1, QVector2D& best_refl_2, QVector2D& best_refl_3,
                                  qreal& best_power)
{
    // Base case: we've selected 3 walls
    if (walls_used.size() == 3) {
        QVector2D refl_1, refl_2, refl_3;
        qreal power = tryReflectionPath(transmitter, receiver, 
                                        walls_used[0], walls_used[1], walls_used[2],
                                        refl_1, refl_2, refl_3);
        
        if (power > best_power) {
            best_power = power;
            best_refl_1 = refl_1;
            best_refl_2 = refl_2;
            best_refl_3 = refl_3;
        }
        return;
    }
    
    // Recursive case: try each remaining wall
    for (int i = 0; i < available_walls.size(); i++) {
        Obstacle* wall = available_walls[i];
        
        // Create new list without this wall (to avoid reusing same wall)
        QList<Obstacle*> remaining_walls;
        for (int j = 0; j < available_walls.size(); j++) {
            if (j != i) {
                remaining_walls.append(available_walls[j]);
            }
        }
        
        // Add this wall to used list and recurse
        QList<Obstacle*> new_used = walls_used;
        new_used.append(wall);
        
        tryAllReflectionCombinations(transmitter, receiver, remaining_walls, new_used,
                                     best_refl_1, best_refl_2, best_refl_3, best_power);
    }
}

/**
 * Compute received power for triple-reflected ray (recursive approach)
 */
qreal computeSimulatedTripleReflectionPower(Transmitter* transmitter, Receiver* receiver,
                                             Obstacle* wall_1, Obstacle* wall_2, Obstacle* wall_3, Obstacle* wall_4,
                                             QVector2D& refl_1_out, QVector2D& refl_2_out,
                                             QVector2D& refl_3_out)
{
    QVector2D tx_pos(transmitter->x(), transmitter->y());
    QVector2D rx_pos(receiver->x(), receiver->y());
    
    qDebug() << "\n--- SIMULATED CALCULATION (TRIPLE REFLECTION - RECURSIVE) ---";
    qDebug() << "TX:" << tx_pos.x() << "," << tx_pos.y();
    qDebug() << "RX:" << rx_pos.x() << "," << rx_pos.y();
    
    // Prepare list of available walls
    QList<Obstacle*> available_walls;
    available_walls.append(wall_1);
    available_walls.append(wall_2);
    available_walls.append(wall_3);
    available_walls.append(wall_4);
    
    qreal best_power = 0.0;
    QVector2D best_refl_1, best_refl_2, best_refl_3;
    
    // Try all combinations recursively
    QList<Obstacle*> walls_used;
    tryAllReflectionCombinations(transmitter, receiver, available_walls, walls_used,
                                 best_refl_1, best_refl_2, best_refl_3, best_power);
    
    if (best_power > 0.0) {
        refl_1_out = best_refl_1;
        refl_2_out = best_refl_2;
        refl_3_out = best_refl_3;
        
        qDebug() << "Best reflection found:";
        qDebug() << "Reflection point 1:" << best_refl_1.x() << "," << best_refl_1.y();
        qDebug() << "Reflection point 2:" << best_refl_2.x() << "," << best_refl_2.y();
        qDebug() << "Reflection point 3:" << best_refl_3.x() << "," << best_refl_3.y();
        qDebug() << "P_RX (simulated):" << best_power << "W";
    } else {
        qDebug() << "No valid triple reflection found";
    }
    
    return best_power;
}

// ============================================================================
// GRAPHICS & VISUALIZATION
// ============================================================================

/**
 * Create graphics scene with all walls
 */
QGraphicsScene* createGraphicsSceneWithWallsTR()
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

    // Wall 4: Additional wall for three reflections
    wall4 = new Obstacle(QVector2D(0, 70), QVector2D(100, 70), GenericWall, 0.4);
    scene->addItem(wall4->graphics);
    qDebug() << "Wall 4 created (horizontal at top)";

    return scene;
}

/**
 * Draw triple-reflected ray path with specified color
 */
void drawTripleReflectedRay(QGraphicsScene* scene, Transmitter* transmitter, Receiver* receiver,
                            const QVector2D& refl_1, const QVector2D& refl_2, const QVector2D& refl_3,
                            QColor ray_color = Qt::cyan)
{
    if (!scene || !transmitter || !receiver) return;
    
    QPen ray_pen(ray_color);
    ray_pen.setWidthF(2);
    
    // TX to first reflection
    QGraphicsLineItem* ray1 = scene->addLine(
        10 * transmitter->x(), 10 * transmitter->y(),
        10 * refl_1.x(), 10 * refl_1.y()
    );
    ray1->setPen(ray_pen);
    
    // First to second reflection
    QGraphicsLineItem* ray2 = scene->addLine(
        10 * refl_1.x(), 10 * refl_1.y(),
        10 * refl_2.x(), 10 * refl_2.y()
    );
    ray2->setPen(ray_pen);
    
    // Second to third reflection
    QGraphicsLineItem* ray3 = scene->addLine(
        10 * refl_2.x(), 10 * refl_2.y(),
        10 * refl_3.x(), 10 * refl_3.y()
    );
    ray3->setPen(ray_pen);
    
    // Third reflection to RX
    QGraphicsLineItem* ray4 = scene->addLine(
        10 * refl_3.x(), 10 * refl_3.y(),
        10 * receiver->x(), 10 * receiver->y()
    );
    ray4->setPen(ray_pen);
    
    // Mark reflection points
    QList<QVector2D> points = {refl_1, refl_2, refl_3};
    for (int i = 0; i < 3; i++) {
        QVector2D point = points[i];
        QGraphicsEllipseItem* marker = scene->addEllipse(
            10 * point.x() - 3, 10 * point.y() - 3, 6, 6
        );
        QPen marker_pen(ray_color);
        marker_pen.setWidthF(1);
        marker->setPen(marker_pen);
        marker->setBrush(QBrush(ray_color));
    }
}

// ============================================================================
// VALIDATION TEST CASE
// ============================================================================

void testCase_TripleReflection()
{
    qDebug() << "\n========== TRIPLE REFLECTION TEST CASE (ALL PERMUTATIONS) ==========";

    qreal tx_power = 0.1;
    Transmitter* tx = new Transmitter(10.0, 5.2, 0, "TX_Test");
    Receiver* rx = new Receiver(6.8, 13.3, 0.5, false);

    tx->power = tx_power;
    tx->gain = G_TX;
    rx->gain = G_TX;

    QVector2D tx_pos(tx->x(), tx->y());
    QVector2D rx_pos(rx->x(), rx->y());

    if (validation_scene) {
        addTransmitterToScene(validation_scene, tx);
        addReceiverToScene(validation_scene, rx);
    }

    QList<Obstacle*> all_walls = {wall1, wall2, wall3, wall4};
    QList<QColor> colors = {Qt::cyan, Qt::yellow, Qt::magenta, Qt::green, 
                            Qt::red, Qt::blue, Qt::white, Qt::gray,
                            Qt::darkCyan, Qt::darkYellow, Qt::darkMagenta, Qt::darkGreen,
                            Qt::lightGray, Qt::darkGray, Qt::darkRed, Qt::darkBlue,
                            Qt::darkGreen, Qt::darkRed, Qt::darkBlue, Qt::darkCyan,
                            Qt::cyan, Qt::yellow, Qt::magenta, Qt::green};

    int valid_count = 0;
    int perm_idx = 0;

    // Generate all permutations: choose 3 walls in order from 4 walls
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            if (j == i) continue; // Skip if same wall
            for (int k = 0; k < 4; k++) {
                if (k == i || k == j) continue; // Skip if same wall
                
                Obstacle* w1 = all_walls[i];
                Obstacle* w2 = all_walls[j];
                Obstacle* w3 = all_walls[k];
                
                QString wall_names;
                if (w1 == wall1) wall_names += "1"; else if (w1 == wall2) wall_names += "2"; else if (w1 == wall3) wall_names += "3"; else wall_names += "4";
                wall_names += "-";
                if (w2 == wall1) wall_names += "1"; else if (w2 == wall2) wall_names += "2"; else if (w2 == wall3) wall_names += "3"; else wall_names += "4";
                wall_names += "-";
                if (w3 == wall1) wall_names += "1"; else if (w3 == wall2) wall_names += "2"; else if (w3 == wall3) wall_names += "3"; else wall_names += "4";
                
                qDebug() << "\n[Perm" << perm_idx + 1 << "] Testing walls:" << wall_names;
                
                QVector2D refl_1, refl_2, refl_3;
                qreal P_simulated = tryReflectionPath(tx, rx, w1, w2, w3, refl_1, refl_2, refl_3);
                
                if (P_simulated <= 0.0) {
                    qDebug() << "✗ No valid triple reflection for walls" << wall_names;
                } else {
                    valid_count++;
                    qDebug() << "✓ Found valid triple reflection for walls:" << wall_names;
                    
                    qreal gamma_mag = computeReflectionCoeff_Magnitude_TR(0.0);
                    qreal P_theoretical = computeTheoreticalPower_TripleReflection(
                        tx_pos, rx_pos, refl_1, refl_2, refl_3, tx->power, tx->gain, rx->gain, 
                        gamma_mag, gamma_mag, gamma_mag
                    );
                    
                    qreal P_theoretical_dBm = powerW_to_dBm(P_theoretical);
                    qreal P_simulated_dBm = powerW_to_dBm(P_simulated);
                    
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
                        drawTripleReflectedRay(validation_scene, tx, rx, refl_1, refl_2, refl_3, colors[perm_idx % colors.size()]);
                    }
                }
                
                perm_idx++;
            }
        }
    }
    
    qDebug() << "\n========== ALL 24 PERMUTATIONS TESTED ==========";
    qDebug() << "Valid reflections found:" << valid_count << "/ 24";
}

// ============================================================================
// MAIN VALIDATION RUNNER
// ============================================================================

/**
 * Run triple reflection validation test
 */
QGraphicsView* runTripleReflectionValidation()
{
    qDebug() << "\n\n";
    qDebug() << "╔════════════════════════════════════════════════════════════╗";
    qDebug() << "║     TRIPLE-REFLECTION PROPAGATION VALIDATION               ║";
    qDebug() << "╚════════════════════════════════════════════════════════════╝";

    qDebug() << "\nTest Parameters:";
    qDebug() << "  Frequency:" << freq / 1e9 << "GHz";
    qDebug() << "  Wavelength:" << lambda << "m";
    qDebug() << "  TX Power:" << P_TX * 1000 << "mW =" << P_TX_dBm << "dBm";
    qDebug() << "  TX/RX Gain:" << G_TX << "(linear)";
    qDebug() << "  Material: Concrete (ε_r = " << epsilon_r << ")";

    validation_scene = createGraphicsSceneWithWallsTR();
    qDebug() << "\nGraphics scene created";

    testCase_TripleReflection();

    qDebug() << "\n════════════════════════════════════════════════════════════";
    qDebug() << "TRIPLE-REFLECTION VALIDATION COMPLETE\n";

    QGraphicsView* view = displayVisualization(validation_scene, 3);
    return view;
}