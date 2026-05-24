#include "validate_common_utils.h"

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QPushButton>
#include <QFileDialog>
#include <QVBoxLayout>

void showValidationScene(QGraphicsScene* scene, const QString& title) {
    QGraphicsView* view = new QGraphicsView(scene);
    view->setRenderHint(QPainter::Antialiasing);
    // invert y-axis to have it point up ?
    view->scale(1, -1);

    QWidget* window = new QWidget();
    window->setWindowTitle(title);
    window->resize(1000, 600);
    window->setAttribute(Qt::WA_DeleteOnClose);

    QVBoxLayout* layout = new QVBoxLayout(window);
    layout->addWidget(view);

    QPushButton* btnSave = new QPushButton("Save Scene as PNG");
    layout->addWidget(btnSave);

    QObject::connect(btnSave, &QPushButton::clicked, [window, view]() {
        QString fileName = QFileDialog::getSaveFileName(window, "Save Scene", "", "PNG Image (*.png)");
        if (!fileName.isEmpty()) {
            QPixmap pixmap(view->viewport()->size());
            view->viewport()->render(&pixmap);
            pixmap.save(fileName);
        }
    });

    window->show();
}

