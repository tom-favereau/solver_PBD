#ifndef DRAWAREA_H
#define DRAWAREA_H

#include <QWidget>
#include <QVector>
#include <QTimer>
#include <QElapsedTimer>
#include <memory>
#include <QKeyEvent>

#include "physicalbody.h"
#include "constraints.h"

class DrawArea : public QWidget
{
Q_OBJECT
public:
    explicit DrawArea(QWidget *parent = nullptr);

protected:
    void mousePressEvent(QMouseEvent *event) override;
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;



private slots:
    void animate();
    void emitCenterSphere();


private:
    void initializeGrid(const QSize &initialSize);
    void rebuildGrid(const QSize &newSize);
    void rebuildBoundaryConstraints();
    void insertSphere(const Sphere &sphere);
    void rehashGrid();
    [[nodiscard]] int cellIndexFor(const QPointF &position) const;
    [[nodiscard]] QPoint cellCoords(int cellIndex) const;
    [[nodiscard]] bool isValidCell(int col, int row) const;

    QTimer emissionTimer;
    bool isEmitting = false;

    bool isCenterCellEmpty() const;

    template <typename Func>
    void forEachSphere(Func &&func);

    void integrateBodies(float dt);
    void satisfyStaticConstraints();
    void solveSphereContacts();
    static bool resolveSpherePair(Sphere &a, Sphere &b);
    void updateVelocities(float dt);
    void applyVelocityDamping(float factor);

    QVector<QVector<Sphere>> grid;
    QVector<std::shared_ptr<StaticConstraint>> staticConstraints;

    int gridCols = 1;
    int gridRows = 1;
    float cellWidth = 1.f;
    float cellHeight = 1.f;
    const float targetCellSize = 120.f;

    QTimer timer;
    QElapsedTimer clock;
    qint64 lastTick = 0;
};

template <typename Func>
void DrawArea::forEachSphere(Func &&func)
{
    for (QVector<Sphere> &cell : grid) {
        for (Sphere &sphere : cell) {
            func(sphere);
        }
    }
}

#endif // DRAWAREA_H