#include "drawarea.h"

#include <QMouseEvent>
#include <QPainter>
#include <QPen>
#include <QBrush>
#include <QResizeEvent>
#include <QRandomGenerator>
#include <algorithm>
#include <cmath>


// this is a anonyme space with utils function
namespace {

    inline QPointF vectorToPointF(const QVector2D &v)
    {
        return QPointF(v.x(), v.y());
    }

    inline int clampIndex(int value, qsizetype size)
    {
        if (size <= 0)
            return 0;

        const int maxIndex = static_cast<int>(size - 1);
        return std::clamp(value, 0, maxIndex);
    }

    const QVector2D kGravity(0.f, 1200.f);
    const QColor kConstraintBlue(0, 102, 255, 200);
    inline void drawConstraintCircle(QPainter &p, const QPointF &center, float radius)
    {
        p.drawEllipse(center, radius, radius);
    }



}

DrawArea::DrawArea(QWidget *parent)
        : QWidget(parent)
{
    setFocusPolicy(Qt::StrongFocus); //to use with key binding
    setStyleSheet("background: white;");

    QSize initialSize = size();
    if (initialSize.isEmpty()) {
        initialSize = QSize(800, 600);
        resize(initialSize);
    }

    initializeGrid(initialSize);
    rebuildBoundaryConstraints();

    connect(&timer, &QTimer::timeout, this, &DrawArea::animate);
    timer.start(16);

    emissionTimer.setInterval(120);          // intervalle entre deux sphères (en ms)
    connect(&emissionTimer, &QTimer::timeout, this, &DrawArea::emitCenterSphere);

    clock.start();
    lastTick = clock.nsecsElapsed();
}

void DrawArea::mousePressEvent(QMouseEvent *event)
{
    Sphere sphere;
    sphere.position = event->pos();
    sphere.prevPosition = sphere.position;
    sphere.radius = 30.f;
    sphere.setMass(std::max(1.f, sphere.radius * 0.5f));
    sphere.color = QColor(
            QRandomGenerator::global()->bounded(256),
            QRandomGenerator::global()->bounded(256),
            QRandomGenerator::global()->bounded(256)
    );

    insertSphere(sphere);
    update();
}

void DrawArea::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_E && !event->isAutoRepeat()) {
        if (!isEmitting) {
            isEmitting = true;
            emitCenterSphere();      // émet tout de suite si possible
            emissionTimer.start();
        }
        event->accept();
        return;
    }
    QWidget::keyPressEvent(event);
}

void DrawArea::keyReleaseEvent(QKeyEvent *event)
{

    if (event->key() == Qt::Key_E && !event->isAutoRepeat()) {
        //if (isEmitting) {
        //    emissionTimer.stop();
        //    isEmitting = false;
        //}
        event->accept();
        return;
    }
    QWidget::keyReleaseEvent(event);
}

void DrawArea::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    //painter.setRenderHint(QPainter::Antialiasing, true);

    for (const QVector<Sphere> &cell : grid) {
        for (const Sphere &sphere : cell) {
            QPen pen(sphere.color, 3);
            painter.setPen(pen);
            painter.setBrush(QBrush(sphere.color));
            painter.drawEllipse(sphere.position, sphere.radius, sphere.radius);
        }
    }

    QPen constraintPen(kConstraintBlue, 2);
    constraintPen.setStyle(Qt::DashLine);
    constraintPen.setCosmetic(true);
    painter.setPen(constraintPen);
    painter.setBrush(QBrush(QColor(0, 102, 255, 40)));
    // add
    for (const auto &constraint : staticConstraints) {
        if (!constraint)
            continue;

        if (auto sphereConstraint = std::dynamic_pointer_cast<SphereConstraint>(constraint)) {
            drawConstraintCircle(painter, sphereConstraint->center(), sphereConstraint->radius());
        } else if (auto bowlConstraint = std::dynamic_pointer_cast<BowlConstraint>(constraint)) {
            drawConstraintCircle(painter, bowlConstraint->center(), bowlConstraint->radius());
        }
    }
}

void DrawArea::resizeEvent(QResizeEvent *event)
{
    rebuildGrid(event->size());
    rebuildBoundaryConstraints();
    QWidget::resizeEvent(event);
}

void DrawArea::animate()
{
    qint64 now = clock.nsecsElapsed();
    float frameDt = (now - lastTick) * 1e-9f;
    lastTick = now;

    if (frameDt <= 0.f)
        frameDt = 1.f / 60.f;

    frameDt = std::clamp(frameDt, 1.f / 240.f, 1.f / 20.f);

    constexpr int subSteps = 4;
    constexpr int solverIterations = 4;
    constexpr float damping = 1.0f; //0.998f;


    float dt = frameDt / subSteps;

    for (int step = 0; step < subSteps; ++step) {
        integrateBodies(dt);
        rehashGrid();

        for (int iter = 0; iter < solverIterations; ++iter) {
            satisfyStaticConstraints();
            rehashGrid();
            solveSphereContacts();
            rehashGrid();
        }

        updateVelocities(dt);
        applyVelocityDamping(damping);
    }

    update();
}

void DrawArea::initializeGrid(const QSize &initialSize)
{
    int widthPx = std::max(1, initialSize.width());
    int heightPx = std::max(1, initialSize.height());

    gridCols = std::max(1, static_cast<int>(std::ceil(static_cast<float>(widthPx) / targetCellSize)));
    gridRows = std::max(1, static_cast<int>(std::ceil(static_cast<float>(heightPx) / targetCellSize)));

    cellWidth = static_cast<float>(widthPx) / static_cast<float>(gridCols);
    cellHeight = static_cast<float>(heightPx) / static_cast<float>(gridRows);

    grid.clear();
    grid.resize(gridCols * gridRows);
}

void DrawArea::rebuildGrid(const QSize &newSize)
{
    QVector<Sphere> bodies;
    bodies.reserve([&]() {
        int count = 0;
        for (const QVector<Sphere> &cell : grid) {
            count += cell.size();
        }
        return count;
    }());

    for (const QVector<Sphere> &cell : grid) {
        for (const Sphere &sphere : cell) {
            bodies.append(sphere);
        }
    }

    initializeGrid(newSize);

    for (const Sphere &sphere : std::as_const(bodies)) {
        insertSphere(sphere);
    }
}

void DrawArea::rebuildBoundaryConstraints()
{
    staticConstraints.clear();

    float w = static_cast<float>(std::max(1, width()));
    float h = static_cast<float>(std::max(1, height()));

    staticConstraints.append(std::make_shared<PlaneConstraint>(QVector2D(1.f, 0.f), 0.f));          // gauche
    staticConstraints.append(std::make_shared<PlaneConstraint>(QVector2D(-1.f, 0.f), -w));          // droite
    staticConstraints.append(std::make_shared<PlaneConstraint>(QVector2D(0.f, 1.f), 0.f));          // haut
    staticConstraints.append(std::make_shared<PlaneConstraint>(QVector2D(0.f, -1.f), -h));          // bas

    //QPointF pillarCenter(w * 0.25f, h * 0.35f);
    //float pillarRadius = std::min(w, h) * 0.1f;
    //staticConstraints.append(std::make_shared<SphereConstraint>(pillarCenter, pillarRadius));

    QPointF bowlCenter(w * 0.5f, h * 0.3f);
    float bowlRadius = std::max(w, h) * 0.5f;
    staticConstraints.append(std::make_shared<BowlConstraint>(bowlCenter, bowlRadius));
}

void DrawArea::insertSphere(const Sphere &sphere)
{
    if (grid.isEmpty())
        return;

    int index = cellIndexFor(sphere.position);
    index = clampIndex(index, grid.size());
    grid[index].append(sphere);
}

void DrawArea::rehashGrid()
{
    if (grid.isEmpty())
        return;

    QVector<QVector<Sphere>> newGrid(grid.size());

    for (const auto &cell : grid) {
        for (const Sphere &sphere : cell) {
            int idx = cellIndexFor(sphere.position);
            idx = clampIndex(idx, newGrid.size());
            newGrid[idx].append(sphere);
        }
    }

    grid.swap(newGrid);
}

int DrawArea::cellIndexFor(const QPointF &position) const
{
    if (gridCols <= 0 || gridRows <= 0)
        return 0;

    float widthF = static_cast<float>(std::max(1, width()));
    float heightF = static_cast<float>(std::max(1, height()));

    float clampedX = std::clamp(static_cast<float>(position.x()), 0.f, widthF - 1e-3f);
    float clampedY = std::clamp(static_cast<float>(position.y()), 0.f, heightF - 1e-3f);

    float cw = (cellWidth > 0.f) ? cellWidth : widthF;
    float ch = (cellHeight > 0.f) ? cellHeight : heightF;

    int col = std::clamp(static_cast<int>(clampedX / cw), 0, gridCols - 1);
    int row = std::clamp(static_cast<int>(clampedY / ch), 0, gridRows - 1);

    return row * gridCols + col;
}

QPoint DrawArea::cellCoords(int cellIndex) const
{
    if (gridCols <= 0)
        return QPoint(0, 0);

    int row = cellIndex / gridCols;
    int col = cellIndex % gridCols;
    return QPoint(col, row);
}

bool DrawArea::isValidCell(int col, int row) const
{
    return col >= 0 && col < gridCols && row >= 0 && row < gridRows;
}

void DrawArea::integrateBodies(float dt)
{
    forEachSphere([dt](Sphere &sphere) {
        if (sphere.invMass <= 0.f)
            return;

        sphere.velocity += kGravity * dt;
        sphere.prevPosition = sphere.position;
        sphere.position += QPointF(sphere.velocity.x() * dt, sphere.velocity.y() * dt);
    });
}

void DrawArea::satisfyStaticConstraints()
{
    for (const auto &constraint : staticConstraints) {
        if (!constraint)
            continue;

        forEachSphere([&](Sphere &sphere) {
            constraint->project(sphere);
        });
    }
}

void DrawArea::solveSphereContacts()
{
    if (gridCols <= 0 || gridRows <= 0)
        return;

    static const QPoint neighborOffsets[] = {
            QPoint(1, 0),
            QPoint(0, 1),
            QPoint(1, 1),
            QPoint(-1, 1)
    };

    for (int row = 0; row < gridRows; ++row) {
        for (int col = 0; col < gridCols; ++col) {
            int index = row * gridCols + col;
            auto &cell = grid[index];

            for (int i = 0; i < cell.size(); ++i) {
                for (int j = i + 1; j < cell.size(); ++j) {
                    resolveSpherePair(cell[i], cell[j]);
                }
            }

            for (const QPoint &offset : neighborOffsets) {
                int neighborCol = col + offset.x();
                int neighborRow = row + offset.y();
                if (!isValidCell(neighborCol, neighborRow))
                    continue;

                auto &neighborCell = grid[neighborRow * gridCols + neighborCol];
                for (Sphere &a : cell) {
                    for (Sphere &b : neighborCell) {
                        resolveSpherePair(a, b);
                    }
                }
            }
        }
    }
}

bool DrawArea::resolveSpherePair(Sphere &a, Sphere &b)
{
    QVector2D delta = QVector2D(b.position - a.position);
    float dist = delta.length();
    float minDist = a.radius + b.radius;

    if (dist >= minDist)
        return false;

    if (dist < 1e-6f) {
        delta = QVector2D(1.f, 0.f);
        dist = 1.f;
    }

    float totalInvMass = a.invMass + b.invMass;
    if (totalInvMass <= 0.f)
        return false;

    float penetration = minDist - dist;
    QVector2D normal = delta / dist;
    QVector2D correction = normal * penetration;

    float shareA = (totalInvMass > 0.f) ? (a.invMass / totalInvMass) : 0.f;
    float shareB = (totalInvMass > 0.f) ? (b.invMass / totalInvMass) : 0.f;

    a.position -= QPointF(correction.x() * shareA, correction.y() * shareA);
    b.position += QPointF(correction.x() * shareB, correction.y() * shareB);

    return true;
}

void DrawArea::updateVelocities(float dt)
{
    if (dt <= 0.f)
        return;

    forEachSphere([dt](Sphere &sphere) {
        QVector2D delta = QVector2D(sphere.position - sphere.prevPosition);
        sphere.velocity = delta / dt;
    });
}

void DrawArea::applyVelocityDamping(float factor)
{
    forEachSphere([factor](Sphere &sphere) {
        sphere.velocity *= factor;
    });
}

bool DrawArea::isCenterCellEmpty() const
{
    if (grid.isEmpty())
        return true;

    QPointF center(width() * 0.5f, height() * 0.5f);
    int idx = clampIndex(cellIndexFor(center), grid.size());
    return grid[idx].isEmpty();
}

void DrawArea::emitCenterSphere()
{
    //if (!isCenterCellEmpty()) {
    //    emissionTimer.stop();
    //    isEmitting = false;
    //    return;
    //}

    Sphere sphere;
    sphere.radius = 5.f;
    sphere.position = QPointF(width() * 0.5f, height() * 0.5f);
    sphere.prevPosition = sphere.position;
    sphere.setMass(1.f);
    sphere.color = QColor(
            QRandomGenerator::global()->bounded(256),
            QRandomGenerator::global()->bounded(256),
            QRandomGenerator::global()->bounded(256)
    );

    const float k = 220.f;                             // intensité de la vitesse
    float t = static_cast<float>(clock.nsecsElapsed()) * 1e-9f;
    sphere.velocity = QVector2D(std::cos(t) * k, k);

    insertSphere(sphere);
    update();
}