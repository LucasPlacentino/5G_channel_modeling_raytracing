#include "validation_recursive.h"
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

extern Obstacle* wall1;
extern Obstacle* wall2;
extern Obstacle* wall3;
extern Obstacle* wall4;

/**
 * Mirror a point through a wall
 */
QVector2D mirrorPoint_Rec(const QVector2D& point, Obstacle* wall)
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
bool findReflectionPoint_Rec(const QVector2D& _start, const QVector2D& _end, 
                            Obstacle* wall, QVector2D& reflection_point_out)
{
    QVector2D d = _end - _start;
    QVector2D x0 = QVector2D(wall->line.p1());
    
    qreal denominator = (wall->unitary.x() * d.y() - wall->unitary.y() * d.x());
    if (std::abs(denominator) < 1e-6) {
        return false;
    }
    
    // t is the parameter along the wall segment
    qreal t = ((d.y() * (_start.x() - x0.x())) - (d.x() * (_start.y() - x0.y()))) / denominator;
    
    qreal wall_length = wall->line.length();
    if (t < 0 || t > wall_length) {
        return false; // Point doesn't lie on the actual finite wall segment
    }
    
    reflection_point_out = x0 + t * wall->unitary;
    
        // u is the parameter along the actual ray from start to end (direction)
    qreal u;
    if (std::abs(d.x()) > std::abs(d.y())) {
        u = (reflection_point_out.x() - _start.x()) / d.x();
    } else {
        u = (reflection_point_out.y() - _start.y()) / d.y();
    }
    
    // The physical reflection MUST simply occur forward in time from current point!
    if (u <= 0.0) {
        return false; // The hit was backwards
    }
    
    return true;
}

/**
 * Compute the path for a given sequence of walls.
 * Returns the list of reflection points. If invalid, returns empty list.
 */
QList<QVector2D> computeGenericPath(const QVector2D& tx, const QVector2D& rx, const QList<Obstacle*>& walls) {
    QList<QVector2D> images;
    QVector2D current_img = rx;
    images.prepend(current_img);

    // Mirror backwards
    for (int i = walls.size() - 1; i >= 0; i--) {
        current_img = mirrorPoint_Rec(current_img, walls[i]);
        images.prepend(current_img);
    }
    
    QList<QVector2D> path;
    QVector2D current_start = tx;
    
    // Trace forwards
    for (int i = 0; i < walls.size(); i++) {
        QVector2D target = images[i + 1];
        QVector2D intersection;
        if (!findReflectionPoint_Rec(current_start, target, walls[i], intersection)) {
            return {}; // invalid intersection
        }
        
        if ((intersection - current_start).length() < 0.01) return {};
        
        path.append(intersection);
        current_start = intersection;
    }
    
    if ((rx - current_start).length() < 0.01) return {};
    return path;
}

/**
 * Draw a generic ray path
 */
void drawGenericRay(QGraphicsScene* scene, Transmitter* transmitter, Receiver* receiver,
                    const QList<QVector2D>& reflection_points, QColor ray_color = Qt::cyan)
{
    if (!scene || !transmitter || !receiver) return;
    
    QPen ray_pen(ray_color);
    ray_pen.setWidthF(1.5);
    
    QVector2D current_start(transmitter->x(), transmitter->y());
    
    for (int i = 0; i < reflection_points.size(); i++) {
        QVector2D next_pt = reflection_points[i];
        scene->addLine(10 * current_start.x(), 10 * current_start.y(),
                       10 * next_pt.x(), 10 * next_pt.y(), ray_pen);
        
        // Draw marker
        QGraphicsEllipseItem* marker = scene->addEllipse(
            10 * next_pt.x() - 3, 10 * next_pt.y() - 3, 6, 6
        );
        marker->setPen(QPen(ray_color));
        marker->setBrush(QBrush(ray_color));
        
        current_start = next_pt;
    }
    
    // Ray to receiver
    scene->addLine(10 * current_start.x(), 10 * current_start.y(),
                   10 * receiver->x(), 10 * receiver->y(), ray_pen);
}

/**
 * Helper recursively building wall sequences up to target max depth
 */
void recursivelyFindPaths(int current_depth, int max_depth, const QList<Obstacle*>& all_walls, const QVector2D& tx, const QVector2D& rx, QList<Obstacle*> current_seq, QList<QList<QVector2D>>& valid_paths) 
{
    if (current_depth > 0) {
        QList<QVector2D> path = computeGenericPath(tx, rx, current_seq);
        if (!path.isEmpty()) {
            valid_paths.append(path);
        }
    }
    
    if (current_depth == max_depth) return;
    
    for (Obstacle* w : all_walls) {
        // Disallow hitting the exact same wall twice in a row (corner reflections okay)
        if (!current_seq.isEmpty() && current_seq.last() == w) continue;
        
        QList<Obstacle*> next_seq = current_seq;
        next_seq.append(w);
        recursivelyFindPaths(current_depth + 1, max_depth, all_walls, tx, rx, next_seq, valid_paths);
    }
}

/**
 * Run recursive validaton test. Default depth is 3. 
 */
void testCase_Recursive(int max_reflections = 3)
{
    qDebug() << "\n========== RECURSIVE REFLECTION TEST CASE (Max:" << max_reflections << "reflections) ==========";

    Transmitter* tx = new Transmitter(10.0, 5.2, 0, "TX_Test");
    Receiver* rx = new Receiver(6.8, 13.3, 0.5, false);

    if (validation_scene) {
        addTransmitterToScene(validation_scene, tx);
        addReceiverToScene(validation_scene, rx);
    }

    QList<Obstacle*> all_walls = {wall1, wall2, wall3, wall4};
    QVector2D tx_pos(tx->x(), tx->y());
    QVector2D rx_pos(rx->x(), rx->y());

    // Compute Direct Path (0 reflections)
    qDebug() << "[0 reflections] Drawing direct path...";
    drawGenericRay(validation_scene, tx, rx, {}, Qt::white);

    // Recursively find all higher order valid rays
    QList<QList<QVector2D>> valid_paths;
    QList<Obstacle*> initial_sequence;
    recursivelyFindPaths(0, max_reflections, all_walls, tx_pos, rx_pos, initial_sequence, valid_paths);

    qDebug() << "Total valid multipath rays found:" << valid_paths.size();

    // Map depths to colors: {1: Cyan, 2: Magenta, 3: Yellow, 4: Green}
    QList<QColor> depth_colors = {Qt::white, Qt::cyan, Qt::magenta, Qt::yellow, Qt::green, Qt::red};

    for (int i = 0; i < valid_paths.size(); i++) {
        int depth = valid_paths[i].size();
        QColor color = (depth < depth_colors.size()) ? depth_colors[depth] : Qt::darkGray;
        if (validation_scene) {
            drawGenericRay(validation_scene, tx, rx, valid_paths[i], color);
        }
    }
}

QGraphicsScene* createGraphicsSceneWithWallsRec()
{
    QGraphicsScene* scene = new QGraphicsScene();
    scene->setSceneRect(-scene_offset, -scene_offset, 1000 + scene_offset*2, 700 + scene_offset*2);
    scene->setBackgroundBrush(QBrush(Qt::black));

    wall1 = new Obstacle(QVector2D(0, 0), QVector2D(50, 0), GenericWall, 0.4);
    scene->addItem(wall1->graphics);
    wall2 = new Obstacle(QVector2D(0, 0), QVector2D(0, 70), GenericWall, 0.4);
    scene->addItem(wall2->graphics);
    wall3 = new Obstacle(QVector2D(50, 0), QVector2D(100, 20), GenericWall, 0.4);
    scene->addItem(wall3->graphics);
    wall4 = new Obstacle(QVector2D(0, 70), QVector2D(100, 70), GenericWall, 0.4);
    scene->addItem(wall4->graphics);

    return scene;
}

QGraphicsView* runRecursiveValidation(int max_reflections)
{
    validation_scene = createGraphicsSceneWithWallsRec();
    testCase_Recursive(max_reflections);
    return displayVisualization(validation_scene, max_reflections);
}
