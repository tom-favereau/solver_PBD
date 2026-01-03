//
// Created by Tom Favereau on 03/01/2026.
//

#include "context.h"


namespace
{
    inline int clampIndex(int value, qsizetype size)
    {
        if (size <= 0)
            return 0;

        const int maxIndex = static_cast<int>(size - 1);
        return std::clamp(value, 0, maxIndex);
    }
}



void Context::initialize(const QSize &initialSize)
{
    QSize size = initialSize.isEmpty() ? QSize(800, 600) : initialSize;
    sceneSize_ = size;
    initializeGrid(size);
    rebuildStaticConstraints();
}

void Context::resizeScene(const QSize &newSize)
{
    if (newSize.isEmpty())
        return;

    sceneSize_ = newSize;
    rebuildGrid(newSize);
    rebuildStaticConstraints();
}

void Context::step(float frameDt)
{
    if (frameDt <= 0.f)
        return;

    const float dt = frameDt / static_cast<float>(subSteps);

    for (int stepIndex = 0; stepIndex < subSteps; ++stepIndex) {
        solver::integrateBodies(grid_, dt);
        updateGrid();

        for (int iter = 0; iter < solverIterations; ++iter) {
            solver::satisfyStaticConstraints(grid_, staticConstraints);
            updateGrid();

            solver::satisfySpringConstraints(grid_, springLinks, subSteps);
            updateGrid();

            solver::solveSphereContacts(
                    grid_,
                    gridCols,
                    gridRows,
                    [this](int col, int row) { return isValidCell(col, row); });
            updateGrid();
        }

        solver::updateVelocities(grid_, dt);
        solver::applyVelocityDamping(grid_, dampingFactor);
    }
}

void Context::addUserSphere(const QPointF &position)
{
    Sphere sphere;
    sphere.position = position;
    sphere.prevPosition = sphere.position;
    sphere.radius = 30.f;
    sphere.setMass(std::max(1.f, sphere.radius * 0.5f));
    sphere.color = QColor(
            QRandomGenerator::global()->bounded(256),
            QRandomGenerator::global()->bounded(256),
            QRandomGenerator::global()->bounded(256));

    insertSphere(sphere);
}

void Context::emitCenterSphere(float timeSeconds)
{
    Sphere sphere;
    sphere.radius = 5.f;
    sphere.position = sceneCenter();
    sphere.prevPosition = sphere.position;
    sphere.setMass(1.f);
    sphere.color = QColor(
            QRandomGenerator::global()->bounded(256),
            QRandomGenerator::global()->bounded(256),
            QRandomGenerator::global()->bounded(256));

    const float k = 220.f;
    sphere.velocity = QVector2D(std::cos(timeSeconds) * k, k);

    insertSphere(sphere);
}

void Context::createSpringCluster(const QPointF &center)
{
    const float radius      = 18.f;
    const float halfSpacing = 26.f;
    const float mass        = 6.f;
    const QColor color(220, 80, 80);

    const int clusterId = nextGroupId++;

    struct NodeSpec { QPointF offset; int node{}; };
    const NodeSpec nodes[4] = {
            { QPointF(0.0, -halfSpacing), 0 },
            { QPointF(halfSpacing, 0.0),  1 },
            { QPointF(0.0, halfSpacing),  2 },
            { QPointF(-halfSpacing, 0.0), 3 }
    };

    for (const NodeSpec &spec : nodes) {
        Sphere sphere;
        sphere.radius = radius;
        sphere.position = center + spec.offset;
        sphere.prevPosition = sphere.position;
        sphere.setMass(mass);
        sphere.color = color;
        sphere.groupId = clusterId;
        sphere.nodeIndex = spec.node;

        insertSphere(sphere);
    }

    auto addSpring = [&](int aNode, int bNode, float stiffness = 0.92f) {
        SpringLink spring;
        spring.groupId = clusterId;
        spring.aNode = aNode;
        spring.bNode = bNode;

        QPointF posA = center + nodes[aNode].offset;
        QPointF posB = center + nodes[bNode].offset;
        spring.restLength = QVector2D(posB - posA).length();
        spring.stiffness = stiffness;

        springLinks.append(spring);
    };

    addSpring(0, 1);
    addSpring(1, 2);
    addSpring(2, 3);
    addSpring(3, 0);
    addSpring(0, 2, 0.95f);
    addSpring(1, 3, 0.95f);
}

bool Context::isCenterCellEmpty() const
{
    if (grid_.isEmpty())
        return true;

    int idx = clampIndex(cellIndexFor(sceneCenter()), grid_.size());
    return grid_.cells[idx].isEmpty();
}

QPointF Context::sceneCenter() const
{
    return {static_cast<float>(sceneSize_.width()) * 0.5f,
            static_cast<float>(sceneSize_.height()) * 0.5f};
}

void Context::initializeGrid(const QSize &size)
{
    int widthPx  = std::max(1, size.width());
    int heightPx = std::max(1, size.height());

    gridCols = std::max(1, static_cast<int>(std::ceil(static_cast<float>(widthPx) / targetCellSize)));
    gridRows = std::max(1, static_cast<int>(std::ceil(static_cast<float>(heightPx) / targetCellSize)));

    cellWidth  = static_cast<float>(widthPx) / static_cast<float>(gridCols);
    cellHeight = static_cast<float>(heightPx) / static_cast<float>(gridRows);

    grid_.cells.clear();
    grid_.resize(gridRows, gridCols);
}

void Context::rebuildGrid(const QSize &size)
{
    QVector<Sphere> bodies;
    bodies.reserve([&]() {
        int count = 0;
        for (const QVector<Sphere> &cell : grid_.cells)
            count += static_cast<int>(cell.size());
        return count;
    }());

    for (const QVector<Sphere> &cell : grid_.cells) {
        for (const Sphere &sphere : cell) {
            bodies.append(sphere);
        }
    }

    initializeGrid(size);

    for (const Sphere &sphere : std::as_const(bodies)) {
        insertSphere(sphere);
    }
}

void Context::rebuildStaticConstraints()
{
    staticConstraints.clear();

    float w = static_cast<float>(std::max(1, sceneSize_.width()));
    float h = static_cast<float>(std::max(1, sceneSize_.height()));

    staticConstraints.append(std::make_shared<PlaneConstraint>(QVector2D(1.f, 0.f), 0.f));
    staticConstraints.append(std::make_shared<PlaneConstraint>(QVector2D(-1.f, 0.f), -w));
    staticConstraints.append(std::make_shared<PlaneConstraint>(QVector2D(0.f, 1.f), 0.f));
    staticConstraints.append(std::make_shared<PlaneConstraint>(QVector2D(0.f, -1.f), -h));

    //staticConstraints.append(std::make_shared<SphereConstraint>(QPointF(w * 0.5f, h * 0.8f), std::min(w, h) * 0.1f));

    //staticConstraints.append(std::make_shared<SphereConstraint>(QPointF(w * 0.3f, h * 0.6f), std::min(w, h) * 0.1f));

    //staticConstraints.append(std::make_shared<SphereConstraint>(QPointF(w * 0.7f, h * 0.6f), std::min(w, h) * 0.1f));

    staticConstraints.append(std::make_shared<BowlConstraint>(QPointF(w * 0.5f, h * 0.3f), std::max(w, h) * 0.5f));
}

void Context::insertSphere(const Sphere &sphere)
{
    if (grid_.isEmpty())
        return;

    int index = clampIndex(cellIndexFor(sphere.position), grid_.size());
    grid_.cells[index].append(sphere);
}

void Context::updateGrid()
{
    if (grid_.isEmpty())
        return;

    QVector<QVector<Sphere>> newGrid(grid_.size());

    for (const auto &cell : grid_.cells) {
        for (const Sphere &sphere : cell) {
            int idx = clampIndex(cellIndexFor(sphere.position), newGrid.size());
            newGrid[idx].append(sphere);
        }
    }

    grid_.cells.swap(newGrid);
}

int Context::cellIndexFor(const QPointF &position) const
{
    if (gridCols <= 0 || gridRows <= 0)
        return 0;

    float widthF  = static_cast<float>(std::max(1, sceneSize_.width()));
    float heightF = static_cast<float>(std::max(1, sceneSize_.height()));

    float clampedX = std::clamp(static_cast<float>(position.x()), 0.f, widthF - 1e-3f);
    float clampedY = std::clamp(static_cast<float>(position.y()), 0.f, heightF - 1e-3f);

    float cw = (cellWidth > 0.f) ? cellWidth : widthF;
    float ch = (cellHeight > 0.f) ? cellHeight : heightF;

    int col = std::clamp(static_cast<int>(clampedX / cw), 0, gridCols - 1);
    int row = std::clamp(static_cast<int>(clampedY / ch), 0, gridRows - 1);

    return row * gridCols + col;
}

bool Context::isValidCell(int col, int row) const
{
    return col >= 0 && col < gridCols && row >= 0 && row < gridRows;
}