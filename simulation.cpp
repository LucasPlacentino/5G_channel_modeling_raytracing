#include "simulation.h"

#include <QDir>
#include <QLineSeries>
#include <QtMath>

#include "parameters.h"

// for path loss plot:
#include <QtCharts/QChartView>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QLogValueAxis>
#include <QtCharts/QValueAxis>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFileDialog>
#include <QShortcut>
#include <QtCharts/QLegendMarker>
#include <QtCharts/QAreaSeries>


Simulation::Simulation(bool show) {
    // class constructor
    this->show = show;

    //// createWalls(); // done in Simulation::run()
}

void Simulation::createWalls()
{
    //qDebug("Creating walls...");

    // Delete old obstacles vector objects (avoid memory leak)
    if(!this->obstacles.empty()) {
        // The vector is not empty
        for(auto ptr : std::as_const(this->obstacles)) {
            delete ptr;
        }
        this->obstacles.clear();
    }

    // --- Outdoor Urban Environment (120x70m Layout) ---
    QList<Obstacle*> urban_walls;
    ObstacleType building_material = GenericWall; 
    qreal thickness = 0.5; // 50cm thick exterior concrete walls

    // The layout features a Main Avenue (Y = 30 to 45) and a Cross Street (X = 50 to 70).

    // 1. Building Block 1 (Top-Left): Features a chamfered intersection corner
    urban_walls.append(new Obstacle(QVector2D(0, 30), QVector2D(45, 30), building_material, thickness)); // South facing
    urban_walls.append(new Obstacle(QVector2D(45, 30), QVector2D(50, 25), building_material, thickness)); // Chamfered corner
    urban_walls.append(new Obstacle(QVector2D(50, 25), QVector2D(50, 0), building_material, thickness)); // East facing

    // 2. Building Block 2 (Top-Right): Features a 10m wide, 20m deep alleyway
    urban_walls.append(new Obstacle(QVector2D(70, 0), QVector2D(70, 30), building_material, thickness)); // West facing
    urban_walls.append(new Obstacle(QVector2D(70, 30), QVector2D(90, 30), building_material, thickness)); // South facing (left of alley)
    urban_walls.append(new Obstacle(QVector2D(100, 30), QVector2D(120, 30), building_material, thickness)); // South facing (right of alley)
    
    // Alleyway walls
    urban_walls.append(new Obstacle(QVector2D(90, 30), QVector2D(90, 10), building_material, thickness)); // Alley West wall
    urban_walls.append(new Obstacle(QVector2D(100, 30), QVector2D(100, 10), building_material, thickness)); // Alley East wall
    urban_walls.append(new Obstacle(QVector2D(90, 10), QVector2D(100, 10), building_material, thickness)); // Alley back wall

    // 3. Building Block 3 (Bottom-Left): Standard 90-degree block
    urban_walls.append(new Obstacle(QVector2D(0, 45), QVector2D(50, 45), building_material, thickness)); // North facing
    urban_walls.append(new Obstacle(QVector2D(50, 45), QVector2D(50, 70), building_material, thickness)); // East facing

    // 4. Building Block 4 (Bottom-Right): Features a recessed corner plaza
    urban_walls.append(new Obstacle(QVector2D(85, 45), QVector2D(120, 45), building_material, thickness)); // North facing main
    urban_walls.append(new Obstacle(QVector2D(85, 45), QVector2D(85, 55), building_material, thickness)); // Plaza West boundary
    urban_walls.append(new Obstacle(QVector2D(70, 55), QVector2D(85, 55), building_material, thickness)); // Plaza North boundary
    urban_walls.append(new Obstacle(QVector2D(70, 55), QVector2D(70, 70), building_material, thickness)); // West facing main

    QList<Obstacle*> all_obstacles;

    //all_obstacles.append(boundary_walls);
    //all_obstacles.append(interior_walls);
    all_obstacles.append(urban_walls);
    //this->obstacles = walls;

    this->obstacles = all_obstacles;

//     // ------ DEBUG: remove all walls ------
// #if REMOVE_ALL_WALLS
//     for(auto ptr : this->obstacles) {
//         delete ptr;
//     }
//     this->obstacles.clear();
// #endif
//     // -------------------------------------

    qDebug() << QString::number(this->obstacles.length()) << "Walls created";
}

void Simulation::run(QProgressBar* progress_bar)
{
    // WARNING: CLEAN UP PREVIOUS QT LAYOUTS FIRST before destroying math!
    if (this->view) {
        delete this->view;
        this->view = nullptr;
    }
    if (this->scene) {
        // Deleting the scene will automatically destroy every QGraphicsItem inside it
        delete this->scene; 
        this->scene = nullptr;
    }

    this->timer.start();
    qDebug() << "Simulation::run() - single cell simulation: " << (this->showRaySingleCell);

    qDebug() << "P_TX:" << P_TX << "W," << P_TX_dBm << "dBm";
    qDebug() << "G_TX:" << G_TX;
    qDebug() << "beta:" << beta_0;
    qDebug() << "lambda:" << lambda << "m";
    qDebug() << "frequency:" << freq << "Hz";
    qDebug() << "omega:" << omega << "rad/s";
    qDebug() << "Exclusion zone for not far-field:" << far_field_min_distance << "m";


    for (QList<Receiver*>& cells_line : this->cells) {
        qDeleteAll(cells_line);
    }
    this->cells.clear();

    qDeleteAll(this->obstacles);
    this->obstacles.clear();

    if (this->remove_all_walls) {
        qDebug() << "Running without any wall";
    } else {
        createWalls();
    }

    if (!this->showRaySingleCell) {
        createCellsMatrix();
    } else {
        // single cell simulation
        Receiver* rx = new Receiver(this->singleCellX,this->singleCellY,0.5, true);
        QPen singleCellPen = QPen(Qt::magenta);
        singleCellPen.setWidthF(10*0.3);
        rx->graphics->setPen(singleCellPen);
        this->cells = {{rx}};
    }

    if (this->cells.isEmpty()) {
        qWarning("no cells provided (simulation.cells matrix is empty)");
        throw std::exception();
    }

    for (Transmitter* tx : std::as_const(this->baseStations)) {
        tx->coverage_area_sqm = 0.0; // reset
        int i=0;
        for (QList<Receiver*>& cells_line : this->cells){
            // loops over each line
            for (Receiver* cell : cells_line) {
                computeDirect(cell, *tx, tx->selector_index);
                computeReflections(cell, *tx, tx->selector_index);
                progress_bar->setValue(i/(cells_line.length()*this->cells.length()));
                i++;
            }
        }
    }
    qDebug() << this->cells.length()*this->cells[0].length() << "cells,";// << i << "computed";

    //end of simulation
    this->simulation_time = this->timer.elapsed();
    qDebug() << "Simulation time:" << this->simulation_time << "ms";

    if (this->show) {
        showView();
    }

}

void Simulation::computeReflections(Receiver* _RX, const QVector2D& _TX, int tx_selector_index)
{
    // 1st reflection :
    for (int i=0; i<this->obstacles.length(); i++) {
        Obstacle* wall = this->obstacles[i];

        if (checkSameSideOfWall(wall->normal,_TX,*_RX)) {
            QVector2D _imageTX = computeImage(_TX, wall);
            QVector2D _P_r = calculateReflectionPoint(_imageTX,*_RX,wall);

            // CHECK IF REFLECTION IS ON THE WALL AND NOT ITS EXTENSION:
            RaySegment* test_segment = new RaySegment(_imageTX.x(),_imageTX.y(),_RX->x(),_RX->y());
            bool valid_1st_reflection = checkRaySegmentIntersectsWall(wall, test_segment);
            //delete test_segment->graphics; // fix memory leak
            delete test_segment;

            if (valid_1st_reflection) {
                Ray* ray_1_reflection = new Ray(_TX.toPointF(), _RX->toPointF(), tx_selector_index);
                QList<RaySegment*> ray_segments;
                ray_segments.append(new RaySegment(_TX.x(),_TX.y(),_P_r.x(),_P_r.y()));
                ray_segments.append(new RaySegment(_P_r.x(),_P_r.y(),_RX->x(),_RX->y()));

                ray_1_reflection->segments = ray_segments;
                addReflection(ray_1_reflection,_imageTX,*_RX,wall);
                checkTransmissions(ray_1_reflection,{wall});

                if (this->showRaySingleCell) {
                    this->singleCellSimReflectionPoints.append(_P_r);
                }

                ray_1_reflection->distance = QVector2D(*_RX-_imageTX).length();
                _RX->all_rays.append(ray_1_reflection);
            }

            // 2nd reflection (we always process this if TX and RX are on the same side,
            // even if the 1st reflection was invalid, because the image still exists)
            for (Obstacle* wall_2 : std::as_const(this->obstacles)) {
                if (wall_2 != wall && checkSameSideOfWall(wall_2->normal,_imageTX,*_RX)) {
                    QVector2D _image_imageTX = computeImage(_imageTX,wall_2);
                    QVector2D _P_r_2_last = calculateReflectionPoint(_image_imageTX,*_RX,wall_2);
                    QVector2D _P_r_2_first = calculateReflectionPoint(_imageTX,_P_r_2_last,wall);

                    RaySegment* test_segment_1 = new RaySegment(_image_imageTX.x(),_image_imageTX.y(),_RX->x(),_RX->y());
                    RaySegment* test_segment_2 = new RaySegment(_imageTX.x(),_imageTX.y(),_P_r_2_last.x(),_P_r_2_last.y());

                    bool valid_2nd_reflection = checkRaySegmentIntersectsWall(wall_2, test_segment_1) &&
                                                checkRaySegmentIntersectsWall(wall,test_segment_2);
                    //delete test_segment_1->graphics; // fix memory leak
                    delete test_segment_1;
                    //delete test_segment_2->graphics; // fix memory leak
                    delete test_segment_2;

                    if (valid_2nd_reflection) {
                        Ray* ray_2_reflection = new Ray(_TX.toPointF(),_RX->toPointF(), tx_selector_index);
                        QList<RaySegment*> ray_segments_2;
                        ray_segments_2.append(new RaySegment(_TX.x(),_TX.y(),_P_r_2_first.x(),_P_r_2_first.y()));
                        ray_segments_2.append(new RaySegment(_P_r_2_first.x(),_P_r_2_first.y(),_P_r_2_last.x(),_P_r_2_last.y()));
                        ray_segments_2.append(new RaySegment(_P_r_2_last.x(),_P_r_2_last.y(),_RX->x(),_RX->y()));

                        ray_2_reflection->segments = ray_segments_2;
                        addReflection(ray_2_reflection,_imageTX,_P_r_2_last,wall);
                        addReflection(ray_2_reflection,_image_imageTX,*_RX,wall_2);
                        checkTransmissions(ray_2_reflection,{wall,wall_2});

                        if (this->showRaySingleCell) {
                            this->singleCellSimReflectionPoints.append(_P_r_2_first);
                            this->singleCellSimReflectionPoints.append(_P_r_2_last);
                        }

                        ray_2_reflection->distance = QVector2D(*_RX-_image_imageTX).length();
                        _RX->all_rays.append(ray_2_reflection);
                    }

                    // 3rd reflection
                    for (Obstacle* wall_3 : std::as_const(this->obstacles)) {
                        if (wall_3 != wall_2 && checkSameSideOfWall(wall_3->normal, _image_imageTX, *_RX)) {
                            QVector2D _image_image_imageTX = computeImage(_image_imageTX, wall_3);

                            QVector2D _P_r_3_last = calculateReflectionPoint(_image_image_imageTX, *_RX, wall_3);
                            QVector2D _P_r_3_mid = calculateReflectionPoint(_image_imageTX, _P_r_3_last, wall_2);
                            QVector2D _P_r_3_first = calculateReflectionPoint(_imageTX, _P_r_3_mid, wall);

                            RaySegment* test_segment_3_1 = new RaySegment(_image_image_imageTX.x(), _image_image_imageTX.y(), _RX->x(), _RX->y());
                            RaySegment* test_segment_3_2 = new RaySegment(_image_imageTX.x(), _image_imageTX.y(), _P_r_3_last.x(), _P_r_3_last.y());
                            RaySegment* test_segment_3_3 = new RaySegment(_imageTX.x(), _imageTX.y(), _P_r_3_mid.x(), _P_r_3_mid.y());

                            bool valid_3rd_reflection = checkRaySegmentIntersectsWall(wall_3, test_segment_3_1) &&
                                                        checkRaySegmentIntersectsWall(wall_2, test_segment_3_2) &&
                                                        checkRaySegmentIntersectsWall(wall, test_segment_3_3);

                            //delete test_segment_3_1->graphics; // fix memory leak
                            delete test_segment_3_1;
                            //delete test_segment_3_2->graphics; // fix memory leak
                            delete test_segment_3_2;
                            //delete test_segment_3_3->graphics; // fix memory leak
                            delete test_segment_3_3;

                            if (valid_3rd_reflection) {
                                Ray* ray_3_reflection = new Ray(_TX.toPointF(), _RX->toPointF(), tx_selector_index);
                                QList<RaySegment*> ray_segments_3;
                                ray_segments_3.append(new RaySegment(_TX.x(), _TX.y(), _P_r_3_first.x(), _P_r_3_first.y()));
                                ray_segments_3.append(new RaySegment(_P_r_3_first.x(), _P_r_3_first.y(), _P_r_3_mid.x(), _P_r_3_mid.y()));
                                ray_segments_3.append(new RaySegment(_P_r_3_mid.x(), _P_r_3_mid.y(), _P_r_3_last.x(), _P_r_3_last.y()));
                                ray_segments_3.append(new RaySegment(_P_r_3_last.x(), _P_r_3_last.y(), _RX->x(), _RX->y()));

                                ray_3_reflection->segments = ray_segments_3;

                                addReflection(ray_3_reflection, _imageTX, _P_r_3_mid, wall);
                                addReflection(ray_3_reflection, _image_imageTX, _P_r_3_last, wall_2);
                                addReflection(ray_3_reflection, _image_image_imageTX, *_RX, wall_3);
                                checkTransmissions(ray_3_reflection, {wall, wall_2, wall_3});

                                if (this->showRaySingleCell) {
                                    this->singleCellSimReflectionPoints.append(_P_r_3_first);
                                    this->singleCellSimReflectionPoints.append(_P_r_3_mid);
                                    this->singleCellSimReflectionPoints.append(_P_r_3_last);
                                }

                                ray_3_reflection->distance = QVector2D(*_RX - _image_image_imageTX).length();
                                _RX->all_rays.append(ray_3_reflection);
                            }
                        }
                    }
                }
            }
        }
    }
}

// compute the image position
QVector2D Simulation::computeImage(const QVector2D& _point, Obstacle* wall) {
    // returns the coordinates of _point's image with wall
    QVector2D new_origin = QVector2D(wall->line.p1()); // set origin to point1 of wall
    //qDebug() << "new coords" << wall->line.p1();
    QVector2D _normal = wall->normal; // normal to the wall (is normalized so it is relative to any origin)
    //qDebug() << "normal" << wall->normal;
    QVector2D new_point_coords = _point - new_origin; // initial point in new coordinates relative to point1 of wall
    double _dotProduct = QVector2D::dotProduct(new_point_coords, _normal);
    QVector2D _image_new_coords = new_point_coords - 2 * _dotProduct * _normal; // image point in new coordinates relative to point1 of wall
    QVector2D _image = new_origin + _image_new_coords; // image point in absolute coordinates
    //qDebug() << "image:" << _image.x() << _image.y();

    return _image;
}

// compute the reflection point on the wall
QVector2D Simulation::calculateReflectionPoint(const QVector2D& _start, const QVector2D& _end, Obstacle* wall) {
    // returns the intersection bewteen the line from _start to _end and the wall
    QVector2D d = _end-_start;
    //QVector2D x0(0,0); // TODO: always this ?
    QVector2D x0 = QVector2D(wall->line.p1()); // TODO: FIXME: correct ?
    qreal t = ((d.y()*(_start.x()-x0.x()))-(d.x()*(_start.y()-x0.y()))) / (wall->unitary.x()*d.y()-wall->unitary.y()*d.x());
    QVector2D P_r = x0 + t * wall->unitary;
    return P_r;
}

void Simulation::addReflection(Ray* _ray, const QVector2D& _p1, const QVector2D& _p2, Obstacle* wall){
    // computes the final Gamma coeff for the ray_segment's reflection with this wall, and adds it to this ray's coeffs list
    QVector2D _d = _p2-_p1;
    qreal _cos_theta_i = abs(QVector2D::dotProduct(_d.normalized(),wall->normal));
    qreal _sin_theta_i = sqrt(1.0 - pow(_cos_theta_i,2));
    qreal _sin_theta_t = _sin_theta_i / sqrt(wall->properties.relative_permittivity);
    qreal _cos_theta_t = sqrt(1.0 - pow(_sin_theta_t,2));
    complex<qreal> Gamma_coeff = computeReflectionCoeff(_cos_theta_i,_sin_theta_i,_cos_theta_t,_sin_theta_t, wall);
    //qDebug() << "addReflection, Gamma_coeff:" << Gamma_coeff;
    if (Gamma_coeff.real() != Gamma_coeff.real() || Gamma_coeff.imag() != Gamma_coeff.imag()) {
        qDebug() << "Gamma_coeff = NaN";
        qDebug() << "_d" << _d;
        qDebug() << "_cos_theta_i" << _cos_theta_i;
        qDebug() << "_sin_theta_i" << _sin_theta_i;
        qDebug() << "_sin_theta_t" << _sin_theta_t;
        qDebug() << "_cos_theta_t" << _cos_theta_t;
    }
    _ray->addCoeff(Gamma_coeff);
}

complex<qreal> Simulation::makeTransmission(RaySegment* ray_segment, Obstacle* wall) {
    // computes the final T coeff for the ray_segment's transmission with this wall, and adds it to this ray's coeffs list
    QVector2D _d = QVector2D(ray_segment->p1())-QVector2D(ray_segment->p2());
    qreal _cos_theta_i = abs(QVector2D::dotProduct(_d.normalized(),wall->normal));
    qreal _sin_theta_i = sqrt(1.0 - pow(_cos_theta_i,2));
    qreal _sin_theta_t = _sin_theta_i / sqrt(wall->properties.relative_permittivity);
    qreal _cos_theta_t = sqrt(1.0 - pow(_sin_theta_t,2));

    complex<qreal> T_coeff = computeTransmissionCoeff(_cos_theta_i,_sin_theta_i,_cos_theta_t,_sin_theta_t,wall);
    if (T_coeff.real() != T_coeff.real() || T_coeff.imag() != T_coeff.imag()) {
        qDebug() << "Gamma_coeff = NaN";
        qDebug() << "_d" << _d;
        qDebug() << "_cos_theta_i" << _cos_theta_i;
        qDebug() << "_sin_theta_i" << _sin_theta_i;
        qDebug() << "_sin_theta_t" << _sin_theta_t;
        qDebug() << "_cos_theta_t" << _cos_theta_t;
    }
    return T_coeff;
}
void Simulation::checkTransmissions(Ray* _ray, QList<Obstacle*> _reflection_walls) {
    // checks for every segment in this ray if they intersect a wall (which isn't a wall already used for a reflection by this ray)
    // if so: adds the Transmission coefficient to this ray's coeffs list
    for (RaySegment* ray_segment : std::as_const(_ray->segments)) {
        for (Obstacle* wall : std::as_const(this->obstacles)) {
            //qDebug() << "pwall" << &wall;
            if (!_reflection_walls.contains(wall)) { // is NOT reflection wall
                if (checkRaySegmentIntersectsWall(wall, ray_segment,nullptr)) {
                    complex<qreal> T_coeff = makeTransmission(ray_segment,wall);
                    //qDebug() << "checkTransmission, T_coeff:" << T_coeff;
                    _ray->addCoeff(T_coeff);
                }
            }
        }
    }
}

void Simulation::computeDirect(Receiver* _RX, const QVector2D& _TX, int tx_selector_index)
{
    // Computes the direct ray: checks all walls between RX and TX and adds
    // their computed transmission coefficients to the direct ray list of coeffs
    Ray* direct_ray = new Ray(_TX.toPointF(), _RX->toPointF(), tx_selector_index);
    RaySegment* _direct_line = new RaySegment(_RX->x(), _RX->y(), _TX.x(), _TX.y());
    for (Obstacle* wall : std::as_const(this->obstacles)) {
        QPointF* intersection_point = nullptr; // not used
        if (checkRaySegmentIntersectsWall(wall, _direct_line, intersection_point)) {
            // transmission through this wall, compute the transmission coeff
            complex<qreal> T_coeff = makeTransmission(_direct_line,wall);
            direct_ray->addCoeff(T_coeff);
        }
        //else {
        //    continue;
        //}
    }
    //qDebug() << "ray_direct distance:" << QVector2D(*_RX - _TX).length();
    direct_ray->distance = QVector2D(*_RX-_TX).length();
    //qDebug() << "Ray's (direct) total coeffs:" << direct_ray->getTotalCoeffs();
    direct_ray->segments = {_direct_line};
    _RX->all_rays.append(direct_ray);
}


complex<qreal> Simulation::computeReflectionCoeff(qreal _cos_theta_i, qreal _sin_theta_i, qreal _cos_theta_t, qreal _sin_theta_t, Obstacle* wall)
{
    // returns the reflection coefficient Gamma_m

    // Assume walls are dielectric slabs (depends on the Fresnel reflection coefficient of the interface and the slab's thickness and material properties):
    qreal eps_r = wall->properties.relative_permittivity;
    
    // Normal polarization Fresnel reflection (Gamma_perp)
    complex<qreal> Gamma_perp = (_cos_theta_i - sqrt(eps_r - pow(_sin_theta_i, 2))) /
                                (_cos_theta_i + sqrt(eps_r - pow(_sin_theta_i, 2)));

    // Phase thickness (q)
    qreal q = beta_0 * wall->thickness * sqrt(eps_r - pow(_sin_theta_i, 2));

    // Slab reflection (Gamma_m) !
    complex<qreal> num = Gamma_perp * (1.0 - exp(-2.0 * j * q));
    complex<qreal> den = 1.0 - pow(Gamma_perp, 2) * exp(-2.0 * j * q);
    return num / den;
}

complex<qreal> Simulation::computeTransmissionCoeff(qreal _cos_theta_i, qreal _sin_theta_i, qreal _cos_theta_t, qreal _sin_theta_t, Obstacle* wall)
{
    // returns the transmission coefficient T_m

    // Assume walls are dielectric slabs (depends on the Fresnel reflection coefficient of the interface and the slab's thickness and material properties):
    qreal eps_r = wall->properties.relative_permittivity;
    
    // Normal polarization Fresnel reflection (Gamma_perp)
    complex<qreal> Gamma_perp = (_cos_theta_i - sqrt(eps_r - pow(_sin_theta_i, 2))) /
                                (_cos_theta_i + sqrt(eps_r - pow(_sin_theta_i, 2)));

    // Phase thickness (q)
    qreal q = beta_0 * wall->thickness * sqrt(eps_r - pow(_sin_theta_i, 2));

    // Slab transmission (T_m)
    complex<qreal> num = (1.0 - pow(Gamma_perp, 2)) * exp(-j * q);
    complex<qreal> den = 1.0 - pow(Gamma_perp, 2) * exp(-2.0 * j * q);
    return num / den;
}

bool Simulation::checkSameSideOfWall(const QVector2D& _normal, const QVector2D& _TX, const QVector2D& _RX) {
    // returns true if _TX and _RX are on the same side of the wall (using the wall's normal vector)
    // must be same sign to be true:
    bool res = (QVector2D::dotProduct(_normal,_RX)>0 == QVector2D::dotProduct(_normal,_TX)>0);
    return res;
}

bool Simulation::checkRaySegmentIntersectsWall(const Obstacle* wall, RaySegment* ray_segment, QPointF* intersection_point) {
    // returns true if ray_segment intersects wall
    // the intersection_point pointer's value is set wit hthe intersection point coordinates if they intersect
    int _intersection_type = ray_segment->intersects(wall->line, intersection_point); // also writes to intersection pointer the QPointF
    bool intersects_wall = _intersection_type==1 ? true: false; //0: no intersection (parallel), 1: intersects directly the line segment, 2: intersects the infinite extension of the line
    return intersects_wall;
}

void Simulation::showView()
{
    qDebug() << "Creating graphics view...";
    QGraphicsScene* sim_scene = createGraphicsScene();
    this->scene = sim_scene;
    this->view = new QGraphicsView(sim_scene); // create user's view showing the graphics scene
    this->view->setAttribute(Qt::WA_DeleteOnClose); // close the view when user clicks on the close button (and not just hide it)

    this->view->setAttribute(Qt::WA_AlwaysShowToolTips); //? maybe necessary ?

    //this->view->setFixedSize(990, 720);
    int view_width = (max_x+10)*10; int view_height = (max_y+20)*10;
    this->view->setFixedSize(view_width, view_height); // new map
    //this->view->scale(6, 6);
    this->view->scale(1, 1); // new map
    qDebug() << "Showing graphics view";
    //QIcon _icon = QIcon(QDir::currentPath()+"/icon.png");
    //view->setWindowIcon(QIcon("./assets/icon.png"));
    view->setWindowIcon(QIcon(":/assets/icon.png"));
    this->view->show(); // shows the graphics scene to the user
}

void Simulation::createCellsMatrix()
{
    int max_x_count = ceil(max_x/this->resolution); // -1 ?
    //qDebug() << "Max count of cells X:" << max_x_count;
    int max_y_count = ceil(max_y/this->resolution); // -1 ?
    //qDebug() << "Max count of cells Y:" << max_y_count;

    qInfo() << "Creating cells matrix" << max_x_count << "x" << max_y_count << "...";
    //qDebug() << "cells matrix initial size:" << this->cells.size();
    for (int x_count=0; x_count < max_x_count; x_count++) {
        //qDebug() << "Creating new line of cells_matrix...";
        qreal x = this->resolution/2+(this->resolution*x_count);
        QList<Receiver*> temp_list;
        for (int y_count=0; y_count < max_y_count; y_count++) {
            qreal y = this->resolution/2+(this->resolution*y_count);
            temp_list.append(new Receiver(x,y,this->resolution, this->show_cell_outline));
            //qDebug() << "cells_matrix line"<< x_count << "size:" << temp_list.size();
        }
        this->cells.append(temp_list);
        //qDebug() << "cells_matrix size:" << this->cells.size();
    }
    qDebug() << "cells_matrix created";
}

Transmitter* Simulation::getBaseStation(int index)
{
    if (index < 0 || index >= this->baseStations.length()) {
        qWarning("baseStations index out of range");
        throw std::out_of_range("baseStations index out of range");
    }
    //return &this->baseStations.at(index);
    return this->baseStations.at(index);
}

void Simulation::deleteBaseStation(int index)
{
    if (index > 0 || index < this->baseStations.size())
    {
        this->baseStations.erase(this->baseStations.begin()+index);
    } else if (index == 0)
    {
        qDebug("Cannot delete Base Station 1");
    } else {
        qWarning("deleteBaseStation error: index out of range");
        throw std::out_of_range("deleteBaseStation error: index out of range");
    }
}

QList<Obstacle*>* Simulation::getObstacles()
{
    return &this->obstacles;
}

unsigned int Simulation::getNumberOfObstacles()
{
    return this->obstacles.size();
}

unsigned int Simulation::getNumberOfBaseStations()
{
    return this->baseStations.size();
}

qint64 Simulation::getSimulationTime() const
{
    return this->simulation_time;
}

QGraphicsScene *Simulation::createGraphicsScene()//std::vector<Transmitter>* TX)
{
    // creates the QGraphicsScene (to give to QGraphicsView) and adds all graphics to it
    qDebug() << "Creating graphics scene...";

    QGraphicsScene* scene = new QGraphicsScene();

    //Transmitter* TX = this->baseStations[0];

    for (QList<Receiver*>& cells_line : this->cells) {
        for (Receiver* RX : cells_line) {
            // compute total power and set it in RX
            qreal _rx_power = 0.0;
            //int _best_tx_id = -1;
            Transmitter* _best_tx = nullptr;
            
            for (Transmitter* tx : std::as_const(this->baseStations)) {
                qreal _pwr = RX->computeTotalPower(tx);
                
                // If it has higher power, update the best TX
                if (_pwr > _rx_power) {
                    _rx_power = _pwr;
                    _best_tx = tx; 
                }
            }

            qreal _rx_power_dBm = 10.0 * std::log10(_rx_power * 1000.0);
            RX->power = _rx_power;

            // Only connect if the power is physically valid
            if (_rx_power > 0.0) {
                RX->connected_tx = _best_tx;
                RX->connected_tx_selector_index = _best_tx ? _best_tx->selector_index : -1;
            } else {
                RX->connected_tx = nullptr;
                RX->connected_tx_selector_index = -1;
            }

            // Coverage Area Calculation:
            if (_best_tx != nullptr && _rx_power > 0.0) {
                if (_rx_power_dBm >= min_sensitivity_dBm) {
                    // area of one square cell is resolution * resolution
                    qreal cell_area = resolution * resolution;
                    _best_tx->coverage_area_sqm += cell_area;
                }
            }

            RX->updateBitrateAndColor();
            QBrush _rxBrush = RX->graphics->brush();
            if (this->showRaySingleCell) {
                _rxBrush.setColor(Qt::black);
            } else {
                _rxBrush.setColor(RX->cell_color);
            }
            RX->graphics->setBrush(_rxBrush);

            // Draw RX and add its tooltip
            //qDebug() << "RX power:" << _rx_power << "W," << _rx_power_dBm << "dBm";
            //qDebug() << "RX bitrate:" << RX->bitrate_Mbps << "Mbps";

            if (RX->connected_tx_selector_index != -1) {
                qreal bitrate_Mbps = RX->bitrate_Mbps;
                if (bitrate_Mbps >= 1000) {
                    qreal bitrate_Gbps = bitrate_Mbps/1000;
                    RX->graphics->setToolTip(
                        //QString("Receiver cell:\nx=%1 y=%2\nPower: %3 mW | %4 dBm\nBitrate: %5 Gbps\nDirect ray (%7 segments) coeffs list length: %6").arg(
                        QString("Receiver cell:\nx=%1 y=%2\n(Total Incoherent Power)\nPower: %3 mW | %4 dBm\nBitrate: %5 Gbps\nConnected BS: %6").arg(
                            QString::number(RX->x()),
                            QString::number(RX->y()),
                            QString::number(_rx_power*1000),
                            QString::number(_rx_power_dBm,'f',2),
                            QString::number(bitrate_Gbps,'f',2),
                            QString::number(RX->connected_tx_selector_index)
                            ////QString::number(RX->all_rays.first()->coeffsList.length()),
                            ////QString::number(RX->all_rays.first()->segments.length())
                            ));
                } else {
                    RX->graphics->setToolTip(
                        //QString("Receiver cell:\nx=%1 y=%2\nPower: %3 mW | %4 dBm\nBitrate: %5 Mbps\nDirect ray (%7 segments) coeffs list length: %6").arg(
                        QString("Receiver cell:\nx=%1 y=%2\n(Total Incoherent Power)\nPower: %3 mW | %4 dBm\nBitrate: %5 Mbps\nConnected BS: %6").arg(
                            QString::number(RX->x()),
                            QString::number(RX->y()),
                            QString::number(_rx_power*1000),
                            QString::number(_rx_power_dBm,'f',2),
                            QString::number(bitrate_Mbps),
                            QString::number(RX->connected_tx_selector_index)
                            //QString::number(RX->all_rays.first()->coeffsList.length()),
                            //QString::number(RX->all_rays.first()->segments.length())
                            )
                        );
                }
            } else {
                RX->graphics->setToolTip(
                    //QString("Receiver cell:\nx=%1 y=%2\nPower: %3 mW | %4 dBm\nBitrate: %5 Mbps\nDirect ray (%7 segments) coeffs list length: %6").arg(
                    QString("Receiver cell:\nx=%1 y=%2\nDead Zone (Near-Field / No Signal)").arg(
                        QString::number(RX->x()),
                        QString::number(RX->y())
                        //QString::number(RX->all_rays.first()->coeffsList.length()),
                        //QString::number(RX->all_rays.first()->segments.length())
                        )
                    );
            }
            scene->addItem(RX->graphics);
            //qDebug() << "RX.graphics:" << RX->graphics->rect();
        }
    }
    qDebug() << "All receiver cells added to scene.";

    // Draw all walls in wall_list
    for (Obstacle* wall : std::as_const(this->obstacles)){
        //qDebug() << "Adding wall to scene...";
        scene->addItem(wall->graphics);
    }
    qDebug() << "All walls added to scene.";

    for (Transmitter* TX : std::as_const(this->baseStations)) {
        TX->graphics->setToolTip(
            QString("Transmitter:\nx=%1 y=%2\nGain: %3\nPower: %4W\nCoverage Area: %5 m² (%6\%)").arg(
                QString::number(TX->x()),
                QString::number(TX->y()),
                QString::number(TX->gain),
                QString::number(TX->power),
                QString::number(TX->coverage_area_sqm),
                QString::number(100*(TX->coverage_area_sqm)/(max_x*max_y),'f',2) // percentage of max possible area of map
                )
            );
        //qDebug() << "TX.graphics:" << TX->graphics->rect();
        scene->addItem(TX->graphics);
        qDebug() << "Transmitter added to scene.";
    }

    if (this->showRaySingleCell) {
        // Draw all rays (their segments) from the all_rays list

        for (Ray* ray : std::as_const(this->cells[0][0]->all_rays)) {
            qDebug() << "Drawing ray";
            for (QGraphicsLineItem* segment_graphics : ray->getSegmentsGraphics()) {
                scene->addItem(segment_graphics);
            }
        }

        //// --- TEST only direct ---
        //#ifdef DEBUG_SINGLE_CELL_DIRECT_RAY
        //for (QGraphicsLineItem* gra : this->cells[0][0]->all_rays.first()->getSegmentsGraphics()) {
        //    scene->addItem(gra);
        //}
        //qDebug() << "number of coeffs:" << this->cells[0][0]->all_rays.first()->coeffsList.length();
        //qDebug() << "T_m=" << this->cells[0][0]->all_rays.first()->coeffsList.first().real() << "+j" << this->cells[0][0]->all_rays.first()->coeffsList.first().imag();
        //#endif
        //// ---

        qDebug() << "All rays added to scene.";

        for (QVector2D point : std::as_const(this->singleCellSimReflectionPoints)) {
            qreal width = 0.04;
            QGraphicsEllipseItem* reflection_graphics = new QGraphicsEllipseItem(10*(point.x()-width),10*(point.y()-width),2*width*10,2*width*10);
            reflection_graphics->setPen(QPen(Qt::green));
            reflection_graphics->setBrush(QBrush(Qt::green));
            reflection_graphics->setToolTip("Reflection point");
            scene->addItem(reflection_graphics);
        }

        qDebug() << "All reflection points added to scene.";
    }

    addLegend(scene);

    scene->setBackgroundBrush(QBrush(Qt::black)); // black background

    qDebug() << "Scene created.";

    return scene;
}

void Simulation::addLegend(QGraphicsScene* scene)
{
    qDebug() << "Adding legend to scene";

    QPen legendPen(Qt::white);
    legendPen.setWidthF(1.5);

    QFont legendFont;
    legendFont.setPointSize(13); // Standard readable font size instead of using setScale()
    QFont smallFont;
    smallFont.setPointSize(9);

    if (!this->showRaySingleCell) {

        QLinearGradient gradient;

        //QRectF rect((max_x * 10) / 2.0 - 138.0 / 2.0, (max_y * 10) + 8, 138, 10); // gradient rectangle scene coordinates
        qreal bar_width = 600.0;
        qreal bar_height = 40.0;
        QRectF rect((max_x * 10) / 2.0 - bar_width / 2.0, (max_y * 10) + 40, bar_width, bar_height);

        QGraphicsRectItem* gradient_graphics = new QGraphicsRectItem(rect);

        gradient.setCoordinateMode(QGradient::ObjectBoundingMode);
        gradient.setColorAt(0.0, QColor::fromRgb(255,0,0)); // blue
        gradient.setColorAt(0.25, QColor::fromRgb(255,255,0)); // cyan
        gradient.setColorAt(0.5, QColor::fromRgb(0,255,0)); // green
        gradient.setColorAt(0.75, QColor::fromRgb(0,255,255)); // yellow
        gradient.setColorAt(1.0, QColor::fromRgb(0,0,255)); // red
        gradient.setStart(1.0, 0.0); // start left
        gradient.setFinalStop(0.0, 0.0); // end right

        QBrush gradientBrush(gradient);
        gradient_graphics->setPen(legendPen);
        gradient_graphics->setBrush(gradientBrush);
        scene->addItem(gradient_graphics); // draw gradient rectangle

        qreal rect_width = rect.width();
        for (int i=0; i<5; i++) {
            QGraphicsLineItem* small_line = new QGraphicsLineItem(rect.bottomLeft().x()+0.25*i*rect_width,rect.bottomLeft().y(),rect.bottomLeft().x()+0.25*i*rect_width,rect.bottomLeft().y()+2.0);
            small_line->setPen(legendPen);
            scene->addItem(small_line);
        }

        // new: Function to easily add aligned text
        auto addLegendText = [&](QString text, qreal x, qreal y) {
            QGraphicsTextItem* t = new QGraphicsTextItem(text);
            t->setDefaultTextColor(Qt::white);
            t->setFont(legendFont);
            // Center the text roughly on the X coordinate
            t->setPos(x - (t->boundingRect().width() / 2.0), y);
            scene->addItem(t);
        };

        // Draw small lines and text underneath the gradient
        for (int i=0; i<5; i++) {
            qreal x_pos = rect.bottomLeft().x() + 0.25 * i * bar_width;
            
            QGraphicsLineItem* small_line = new QGraphicsLineItem(
                x_pos, rect.bottomLeft().y(), 
                x_pos, rect.bottomLeft().y() + 8.0);
            small_line->setPen(legendPen);
            scene->addItem(small_line);
        }

        qreal quarter_bitrate_Mbps = pow(10.0, (log10(min_bitrate_Mbps) + 0.25*(log10(max_bitrate_Mbps)-log10(min_bitrate_Mbps))));
        qreal middle_bitrate_Mbps = pow(10.0, (log10(min_bitrate_Mbps) + 0.5*(log10(max_bitrate_Mbps)-log10(min_bitrate_Mbps))));
        qreal three_quarters_bitrate_Mbps = pow(10.0, (log10(min_bitrate_Mbps) + 0.75*(log10(max_bitrate_Mbps)-log10(min_bitrate_Mbps))));

        qreal text_y = rect.bottomLeft().y() + 10.0;
        addLegendText(QString::number(min_bitrate_Mbps)+" Mbps", rect.bottomLeft().x(), text_y);
        addLegendText(QString::number(quarter_bitrate_Mbps, 'f', 0)+" Mbps", rect.bottomLeft().x() + 0.25*bar_width, text_y);
        addLegendText(QString::number(middle_bitrate_Mbps, 'f', 0)+" Mbps", rect.bottomLeft().x() + 0.5*bar_width, text_y);
        addLegendText(QString::number(three_quarters_bitrate_Mbps, 'f', 0)+" Mbps", rect.bottomLeft().x() + 0.75*bar_width, text_y);
        addLegendText(QString::number(max_bitrate_Mbps/1000)+" Gbps", rect.bottomRight().x(), text_y);

    } else {
        QGraphicsTextItem* ray_colors = new QGraphicsTextItem("Green line: Direct ray\nRed line: One-reflection ray\nYellow line: Two-reflections ray\nCyan line: Three-reflections ray");
        ray_colors->setPos((max_x * 10) / 2.0 - 100.0, (max_y * 10) + 20);
        ray_colors->setScale(1);
        ray_colors->setDefaultTextColor(Qt::white);
        scene->addItem(ray_colors);
    }

    // axes legends :
    qreal offset = -15.0; // Push axes slightly further out to fit text
    QGraphicsLineItem* x_line = new QGraphicsLineItem(0, offset, max_x*10, offset);
    QGraphicsLineItem* y_line = new QGraphicsLineItem(offset, 0, offset, max_y*10);
    x_line->setPen(legendPen);
    y_line->setPen(legendPen);
    scene->addItem(x_line);
    scene->addItem(y_line);

    QGraphicsTextItem* x_label = new QGraphicsTextItem("x");
    x_label->setPos(-25.0, offset - 30.0);
    x_label->setFont(legendFont);
    x_label->setDefaultTextColor(Qt::white);
    scene->addItem(x_label);

    QGraphicsTextItem* m_label = new QGraphicsTextItem("[m]");
    m_label->setPos(offset - 35.0, offset - 20.0);
    m_label->setFont(smallFont);
    m_label->setDefaultTextColor(Qt::white);
    scene->addItem(m_label);

    QGraphicsTextItem* y_label = new QGraphicsTextItem("y");
    y_label->setPos(offset - 40.0, -20.0);
    y_label->setFont(legendFont);
    y_label->setDefaultTextColor(Qt::white);
    scene->addItem(y_label);

    // X-Axis markers:
    for (int x = 0; x <= max_x; x++) { 
        qreal tick_length = 2.0; // Small tick for every 1m

        if (x % 10 == 0) {
            tick_length = 6.0; // Large tick for every 10m
        } else if (x % 5 == 0) {
            tick_length = 4.0; // Medium tick for every 5m
        }

        // Draw the tick line
        QGraphicsLineItem* tick_line = new QGraphicsLineItem(x * 10, offset, x * 10, offset - tick_length);
        tick_line->setPen(legendPen);
        scene->addItem(tick_line);

        // Only draw the text label if it is a 10m mark
        if (x % 10 == 0) {
            QGraphicsTextItem* x_index = new QGraphicsTextItem(QString::number(x));
            x_index->setFont(smallFont);
            x_index->setDefaultTextColor(Qt::white);
            // Center text on the tick
            x_index->setPos((x * 10) - (x_index->boundingRect().width() / 2.0), offset - 30.0);
            scene->addItem(x_index);
        }
    }

    // Y-Axis Markers:
    for (int y = 0; y <= max_y; y++) { 
        qreal tick_length = 2.0; // Small tick for every 1m

        if (y % 10 == 0) {
            tick_length = 6.0; // Large tick for every 10m
        } else if (y % 5 == 0) {
            tick_length = 4.0; // Medium tick for every 5m
        }

        // Draw the tick line
        QGraphicsLineItem* tick_line = new QGraphicsLineItem(offset, y * 10, offset - tick_length, y * 10);
        tick_line->setPen(legendPen);
        scene->addItem(tick_line);

        // Only draw the text label if it is a 10m mark
        if (y % 10 == 0) {
            QGraphicsTextItem* y_index = new QGraphicsTextItem(QString::number(y));
            y_index->setFont(smallFont);
            y_index->setDefaultTextColor(Qt::white);
            // Align text to the left of the tick
            y_index->setPos(offset - 8.0 - y_index->boundingRect().width(), (y * 10) - (y_index->boundingRect().height() / 2.0));
            scene->addItem(y_index);
        }
    }

}

// path loss scatter plot
QChartView* Simulation::showPathLossScatterPlot() // ONLY IF 1 BS !
{
    if (this->baseStations.size() != 1) {
        qWarning() << "Path Loss Scatter Plot is only available for exactly 1 Base Station.";
        return nullptr;
    }
    QScatterSeries *scatterSeries = new QScatterSeries();
    scatterSeries->setName("Simulated UE");
    scatterSeries->setMarkerShape(QScatterSeries::MarkerShapeCircle);
    scatterSeries->setMarkerSize(4.0);
    
    // slightly transparent (alpha = 150) so overlapping points create a "density heat" effect
    scatterSeries->setColor(QColor(0, 100, 255, 150)); 
    scatterSeries->setBorderColor(Qt::transparent);

    qreal min_pwr = 0.0;
    qreal max_pwr = -200.0;
    qreal max_dist = 1.0;
    qreal safe_log_zero = 0.001; // Prevents log10(0) math crashes

    qreal tx_pwr_dBm = this->baseStations[0]->getPower_dBm();
    //qreal min_pl_threshold = tx_pwr_dBm - max_sensitivity_dBm;
    //qreal max_pl_threshold = tx_pwr_dBm - min_sensitivity_dBm;

    // Variables for Least Squares Linear Regression
    qreal sum_x = 0;
    qreal sum_y = 0;
    qreal sum_xy = 0;
    qreal sum_x2 = 0;
    int N = 0;
    QVector<QPointF> regression_data;

    // 1. Extract the Data
    for (const QList<Receiver*>& row : std::as_const(this->cells)) {
        for (Receiver* rx : row) {
            // Only plot valid cells that are connected to a transmitter
            if (rx->power > 0.0 && rx->connected_tx != nullptr) {
                
                qreal d = rx->distanceToPoint(*(rx->connected_tx));
                
                // Enforce the far-field minimum distance (1m) so the Log axis doesn't crash on d=0
                if (d < 1.0) d = 1.0;

                qreal p_dBm = 10.0 * std::log10(rx->power * 1000.0); // received power
                //qreal pl_dB = tx_pwr_dBm - p_dBm; // PL

                scatterSeries->append(d, p_dBm); // received power
                //scatterSeries->append(d, pl_dB); // PL

                // Accumulate data for the log-normal regression fit
                // x = 10 * log10(d), y = P_dBm
                qreal x_val = 10.0 * std::log10(d);
                qreal y_val = p_dBm;
                //qreal y_val = pl_dB; // PL

                sum_x += x_val;
                sum_y += y_val;
                sum_xy += (x_val * y_val);
                sum_x2 += (x_val * x_val);
                N++;
                regression_data.append(QPointF(x_val, y_val));

                // Track bounds for dynamic axes
                if (p_dBm < min_pwr) min_pwr = p_dBm;
                if (p_dBm > max_pwr) max_pwr = p_dBm;
                //if (pl_dB < min_pwr) min_pwr = pl_dB; // PL
                //if (pl_dB > max_pwr) max_pwr = pl_dB; // PL

                if (d > max_dist) max_dist = d;
            }
        }
    }

    if (scatterSeries->count() == 0) {
        qDebug() << "No valid data to plot.";
        return nullptr;
    }

    // -- Compute log-normal regression (path loss law)
    QLineSeries *fitSeries = new QLineSeries();
    QLineSeries *sigmaPlusSeries = new QLineSeries();
    QLineSeries *sigmaMinusSeries = new QLineSeries();
    QLineSeries *friisSeries = new QLineSeries();
    if (N > 1) {
        // Calculate slope (m) and intercept (C) for y = mx + C
        qreal m = (N * sum_xy - sum_x * sum_y) / (N * sum_x2 - sum_x * sum_x);
        qreal C = (sum_y - m * sum_x) / N;
        qreal n = -m; // received power, Path Loss Exponent (PLE?)
        //qreal n = m; // PL, Path Loss Exponent (PLE)

        // Calculate Standard Deviation of Shadowing (sigma_L)
        qreal variance_sum = 0.0;
        for (const QPointF& pt : regression_data) {
            qreal predicted_y = C + m * pt.x(); // x is already 10*log10(d)
            variance_sum += std::pow(pt.y() - predicted_y, 2);
        }
        qreal sigma_L = std::sqrt(variance_sum / N);

        // Setup Pens
        QPen fitPen(Qt::red);
        fitPen.setWidth(3);
        fitSeries->setPen(fitPen);
        //fitSeries->setName(QString("Fit: Path loss exponent (n) = %1").arg(n, 0, 'f', 2));
        //fitSeries->setName(QString("Empirical fit: < P_RX(1m) > = %1 dBm, n = %2")
                               //.arg(C, 0, 'f', 1)
        fitSeries->setName(QString("Empirical fit (n = %1)")
                               .arg(n, 0, 'f', 2));

        QPen sigmaPen(Qt::green);
        sigmaPen.setWidth(2);
        sigmaPen.setStyle(Qt::DotLine);
        sigmaPlusSeries->setPen(sigmaPen);
        sigmaMinusSeries->setPen(sigmaPen);
        //sigmaPlusSeries->setName(QString("+/- 1\u03C3 Shadowing (%1 dB)").arg(sigma_L, 0, 'f', 1));
        sigmaPlusSeries->setName(QString("\u00B11\u03C3 (Shadowing Variability \u03C3_L = %1 dB)")
                                     .arg(sigma_L, 0, 'f', 1));

        QPen friisPen;
        friisPen.setColor("gold");
        friisPen.setWidth(2);
        friisPen.setStyle(Qt::DashLine);
        friisSeries->setPen(friisPen);
        friisSeries->setName("Theoretical: Friis Free Space (n=2)");

        // Generate the curve using 100 points so it bends correctly on a linear axis
        int num_points = 100;
        qreal step = (max_dist - 1.0) / num_points;
        // Convert linear gains (1.64) to decibels
        qreal tx_gain_dB = 10.0 * std::log10(G_TX);
        qreal rx_gain_dB = 10.0 * std::log10(G_RX);
        for (int i = 0; i <= num_points; i++) {
            qreal d_val = 1.0 + i * step;
            qreal log_d = 10.0 * std::log10(d_val);

            // Regression Mean & Sigma bands
            qreal p_val_mean = C + m * log_d;
            fitSeries->append(d_val, p_val_mean);
            sigmaPlusSeries->append(d_val, p_val_mean + sigma_L);
            sigmaMinusSeries->append(d_val, p_val_mean - sigma_L);

            // qreal p_val = C - n * 10.0 * std::log10(d_val); // received power
            // //qreal pl_val = C + n * 10.0 * std::log10(d_val); // PL
            // fitSeries->append(d_val, p_val); // received power
            // //fitSeries->append(d_val, pl_val); // PL

            // theoretical Friis equation
            // PRX = PTX + G_TX + G_RX - FSPL
            qreal p_val_friis = tx_pwr_dBm + tx_gain_dB + rx_gain_dB - (20.0 * std::log10((4.0 * M_PI * d_val) / lambda));
            friisSeries->append(d_val, p_val_friis);
        }
    }

    // 2. Build the Chart
    QChart *chart = new QChart();
    qreal x_max_rounded = std::ceil(max_dist / 10.0) * 10.0;

    // -- Add UE Sensitivity Threshold Lines
    QPen thresholdPen(Qt::black);
    thresholdPen.setWidth(1);
    thresholdPen.setStyle(Qt::DashLine);
    // Min Sensitivity Line
    QLineSeries *minLimitSeries = new QLineSeries();
    //minLimitSeries->setName("UE Min Sensitivity Threshold");
    minLimitSeries->setPen(thresholdPen);
    minLimitSeries->append(safe_log_zero, min_sensitivity_dBm);
    minLimitSeries->append(x_max_rounded, min_sensitivity_dBm);
    //minLimitSeries->append(safe_log_zero, min_pl_threshold); // PL
    //minLimitSeries->append(x_max_rounded, min_pl_threshold); // PL
    // Max Sensitivity (Saturation) Line
    QLineSeries *maxLimitSeries = new QLineSeries();
    //maxLimitSeries->setName("UE Max Sensitivity Threshold");
    maxLimitSeries->setPen(thresholdPen);
    maxLimitSeries->append(safe_log_zero, max_sensitivity_dBm);
    maxLimitSeries->append(x_max_rounded, max_sensitivity_dBm);
    //maxLimitSeries->append(safe_log_zero, max_pl_threshold); // PL
    //maxLimitSeries->append(x_max_rounded, max_pl_threshold); // PL

    // -- Add Near-Field Blackout Zone
    QLineSeries *nearFieldLower = new QLineSeries();
    nearFieldLower->append(safe_log_zero, -300.0); // Arbitrary extreme low
    nearFieldLower->append(far_field_min_distance, -300.0);
    QLineSeries *nearFieldUpper = new QLineSeries();
    nearFieldUpper->append(safe_log_zero, 100.0); // Arbitrary extreme high
    nearFieldUpper->append(far_field_min_distance, 100.0);
    QAreaSeries *nearFieldArea = new QAreaSeries(nearFieldUpper, nearFieldLower);
    nearFieldArea->setPen(Qt::NoPen); // No border line
    nearFieldArea->setBrush(QBrush(Qt::darkGray)); // Solid black fill

    // -- Add series to chart
    // "bg" layer
    //chart->addSeries(nearFieldArea); // painted 1st
    // "mid" layer
    chart->addSeries(minLimitSeries); // painted 2nd
    chart->addSeries(maxLimitSeries); // painted 3rd
    // "top" layer
    chart->addSeries(scatterSeries); // painted 4th
    chart->addSeries(friisSeries);
    chart->addSeries(fitSeries);
    chart->addSeries(sigmaPlusSeries);
    chart->addSeries(sigmaMinusSeries);
    //
    chart->addSeries(nearFieldArea);

    chart->setTitle(QString("Path Loss Scatter Plot (Power vs. Distance), TX power: %1 dBm").arg(QString::number(this->baseStations[0]->getPower_dBm())));

    // -- Larger Chart Title
    QFont titleFont = chart->titleFont();
    titleFont.setPointSize(16);
    titleFont.setBold(true);
    chart->setTitleFont(titleFont);

    chart->legend()->setVisible(true);
    //chart->legend()->setAlignment(Qt::AlignBottom);

    // -- hide threshold lines from legend
    const auto minMarkers = chart->legend()->markers(minLimitSeries);
    for (QLegendMarker *marker : minMarkers) {
        marker->setVisible(false);
    }
    const auto maxMarkers = chart->legend()->markers(maxLimitSeries);
    for (QLegendMarker *marker : maxMarkers) {
        marker->setVisible(false);
    }
    // -- hide near-field bg box
    const auto nearFieldMarkers = chart->legend()->markers(nearFieldArea);
    for (QLegendMarker *marker : nearFieldMarkers) {
        marker->setVisible(false);
    }
    // -- hide one of the -sigma legend because redundant with sigma (put +/-sigma later in legend)
    const auto sigmaMinusMarkers = chart->legend()->markers(sigmaMinusSeries);
    for (QLegendMarker *marker : sigmaMinusMarkers) {
        marker->setVisible(false);
    }

    QFont font;
    font.setPointSize(12);

    // -- X-Axis (Linear scale for Distance)
    // Round max distance up to the nearest 10 (e.g., 123m becomes 130m)
    QValueAxis *axisX = new QValueAxis();
    axisX->setTitleText("Distance (m)");
    axisX->setTitleFont(font);
    axisX->setRange(0.0, x_max_rounded);
    // Set a major tick (with text) every 10 meters
    axisX->setTickCount(static_cast<int>(x_max_rounded / 10.0) + 1);
    // Add 1 minor tick between majors (creates a visual mark every 5 meters)
    axisX->setMinorTickCount(1);
    axisX->setLabelFormat("%d"); // force integer display
    chart->addAxis(axisX, Qt::AlignBottom);

    nearFieldArea->attachAxis(axisX);
    minLimitSeries->attachAxis(axisX);
    maxLimitSeries->attachAxis(axisX);
    scatterSeries->attachAxis(axisX);
    friisSeries->attachAxis(axisX);
    sigmaPlusSeries->attachAxis(axisX);
    sigmaMinusSeries->attachAxis(axisX);
    fitSeries->attachAxis(axisX);

    // -- Y-Axis (Linear scale for dBm)
    // Round min down to nearest 10, and max up to nearest 10
    //qreal y_min_rounded = std::floor((min_pwr - 5.0) / 10.0) * 10.0;
    //qreal y_max_rounded = std::ceil((max_pwr + 5.0) / 10.0) * 10.0;
    qreal y_min_rounded = std::floor((min_pwr - 0.5) / 10.0) * 10.0;
    qreal y_max_rounded = std::ceil((max_pwr + 0.5) / 10.0) * 10.0;
    QValueAxis *axisY = new QValueAxis();
    axisY->setTitleText("Received Power (dBm)"); // received power
    //axisY->setTitleText("Path Loss (dB)"); // PL
    axisY->setTitleFont(font);
    axisY->setRange(y_min_rounded, y_max_rounded);
    // Set a major tick (with text) every 10 dBm
    int y_tick_count = static_cast<int>((y_max_rounded - y_min_rounded) / 10.0) + 1;
    axisY->setTickCount(y_tick_count);
    // Add 1 minor tick between majors (creates a visual mark every 5 dBm)
    axisY->setMinorTickCount(1);
    axisY->setLabelFormat("%d"); // force integer display
    chart->addAxis(axisY, Qt::AlignLeft);

    nearFieldArea->attachAxis(axisY);
    minLimitSeries->attachAxis(axisY);
    maxLimitSeries->attachAxis(axisY);
    scatterSeries->attachAxis(axisY);
    friisSeries->attachAxis(axisY);
    sigmaPlusSeries->attachAxis(axisY);
    sigmaMinusSeries->attachAxis(axisY);
    fitSeries->attachAxis(axisY);

    // 3. Create the UI Window (reuse image export code from TDL)
    QChartView *chartView = new QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);

    QWidget *window = new QWidget();
    window->setWindowTitle("Path Loss Analysis");
    window->resize(1000, 600);
    window->setAttribute(Qt::WA_DeleteOnClose);

    QVBoxLayout *mainLayout = new QVBoxLayout(window);
    mainLayout->addWidget(chartView, 1);

    QHBoxLayout *buttonLayout = new QHBoxLayout();


    QPushButton *btnSavePng = new QPushButton("Save as PNG");
    btnSavePng->setMinimumHeight(30);

    QPushButton *btnToggleX = new QPushButton("Toggle Log/Linear X-Axis");
    btnToggleX->setMinimumHeight(30);

    buttonLayout->addWidget(btnSavePng);
    buttonLayout->addWidget(btnToggleX);

    buttonLayout->addStretch();
    mainLayout->addLayout(buttonLayout);

    // lambda function
    QObject::connect(btnSavePng, &QPushButton::clicked, [window, chartView]() {
        QString fileName = QFileDialog::getSaveFileName(window, "Save Chart", "", "PNG Image (*.png)");
        if (!fileName.isEmpty()) {
            chartView->grab().save(fileName);
        }
    });

    QShortcut *saveShortcut = new QShortcut(QKeySequence::Save, window);
    // lambda function
    QObject::connect(saveShortcut, &QShortcut::activated, [window, chartView]() {
        QString fileName = QFileDialog::getSaveFileName(window, "Save Chart", "", "PNG Image (*.png)");
        if (!fileName.isEmpty()) chartView->grab().save(fileName);
    });

    // lambda function
    QObject::connect(btnToggleX, &QPushButton::clicked, [chart, scatterSeries, minLimitSeries, maxLimitSeries, nearFieldArea, friisSeries, sigmaPlusSeries, sigmaMinusSeries, fitSeries, font, max_dist, x_max_rounded]() {

        // 1. Get the current X-Axis
        QAbstractAxis *oldAxisX = chart->axes(Qt::Horizontal).at(0); // or .constFirst(); rather than .first()
        bool isCurrentlyLinear = (oldAxisX->type() == QAbstractAxis::AxisTypeCategory || oldAxisX->type() == QAbstractAxis::AxisTypeValue);

        // 2. Detach old axis from ALL series
        nearFieldArea->detachAxis(oldAxisX);
        minLimitSeries->detachAxis(oldAxisX);
        maxLimitSeries->detachAxis(oldAxisX);
        scatterSeries->detachAxis(oldAxisX);
        friisSeries->detachAxis(oldAxisX);
        sigmaPlusSeries->detachAxis(oldAxisX);
        sigmaMinusSeries->detachAxis(oldAxisX);
        fitSeries->detachAxis(oldAxisX);

        // 3. Remove and delete the old axis
        chart->removeAxis(oldAxisX);
        delete oldAxisX;

        // 4. Create and attach the new axis
        if (isCurrentlyLinear) {
            // --- Switch to LOGARITHMIC ---
            QLogValueAxis *newAxisX = new QLogValueAxis();
            newAxisX->setTitleText("Distance (m) [Log Scale]");
            newAxisX->setTitleFont(font);
            newAxisX->setBase(10.0);
            newAxisX->setLabelFormat("%g");

            // add 8 minor ticks for the Log grid look
            newAxisX->setMinorTickCount(8);

            // Log axis cannot start at 0. Start at 1.0 (or your far_field_min_distance).
            newAxisX->setRange(1.0, max_dist + 20.0);

            chart->addAxis(newAxisX, Qt::AlignBottom);

            nearFieldArea->attachAxis(newAxisX);
            minLimitSeries->attachAxis(newAxisX);
            maxLimitSeries->attachAxis(newAxisX);
            scatterSeries->attachAxis(newAxisX);
            friisSeries->attachAxis(newAxisX);
            sigmaPlusSeries->attachAxis(newAxisX);
            sigmaMinusSeries->attachAxis(newAxisX);
            fitSeries->attachAxis(newAxisX);

        } else {
            // --- Switch to LINEAR ---
            QValueAxis *newAxisX = new QValueAxis();
            newAxisX->setTitleText("Distance (m)");
            newAxisX->setTitleFont(font);
            newAxisX->setRange(0.0, x_max_rounded);
            newAxisX->setTickCount(static_cast<int>(x_max_rounded / 10.0) + 1);
            newAxisX->setMinorTickCount(1);
            newAxisX->setLabelFormat("%d");

            chart->addAxis(newAxisX, Qt::AlignBottom);

            nearFieldArea->attachAxis(newAxisX);
            minLimitSeries->attachAxis(newAxisX);
            maxLimitSeries->attachAxis(newAxisX);
            scatterSeries->attachAxis(newAxisX);
            friisSeries->attachAxis(newAxisX);
            sigmaPlusSeries->attachAxis(newAxisX);
            sigmaMinusSeries->attachAxis(newAxisX);
            fitSeries->attachAxis(newAxisX);
        }
    });

    window->show();
    return chartView;
}


