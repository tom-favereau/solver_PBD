//
// Created by Tom Favereau on 03/01/2026.
//

#include "solver.h"


namespace
{
    //constexpr QVector2D kGravity(0.f, 1200.f);
    //constexpr QVector2D kGravity(0.f, 600.f);
    constexpr QVector2D kGravity(0.f, 400.f);
}

void Solver::integrateBodies(Grid &grid, float dt) const
{
    multithreading::forEachSphere(grid, [dt](Sphere &sphere) {
        if (sphere.invMass <= 0.f)
            return;

        //sphere.velocity += kGravity * dt; // gravity does not depend on the mass
        sphere.velocity += kGravity * sphere.mass() * dt; //gravity depend on the mass
        sphere.prevPosition = sphere.position;
        sphere.position += QPointF(sphere.velocity.x() * dt, sphere.velocity.y() * dt);
    });
}

void Solver::satisfyStaticConstraints(Grid &grid,
                                      const QVector<std::shared_ptr<StaticConstraint>> &constraints) const
{
    for (const auto &constraint : constraints) {
        if (!constraint)
            continue;

        multithreading::forEachSphere(grid, [&constraint](Sphere &sphere) {
            constraint->project(sphere);
        });
    }
}

void Solver::satisfySpringConstraints(Grid &grid, QVector<SpringLink> &springLinks) const
{
    for (const SpringLink &spring : springLinks) {
        Sphere *a = findSphereNode(grid, spring.groupId, spring.aNode);
        Sphere *b = findSphereNode(grid, spring.groupId, spring.bNode);
        if (!a || !b)
            continue;

        QVector2D delta = QVector2D(b->position - a->position);
        float dist = delta.length();
        if (dist <= 1e-5f)
            continue;

        float totalInvMass = a->invMass + b->invMass;
        if (totalInvMass <= 0.f)
            continue;

        float diff = (dist - spring.restLength) / dist;
        QVector2D correction = delta * diff * spring.stiffness;

        float shareA = a->invMass / totalInvMass;
        float shareB = b->invMass / totalInvMass;

        a->position += QPointF(correction.x() * shareA, correction.y() * shareA);
        b->position -= QPointF(correction.x() * shareB, correction.y() * shareB);
    }
}

void Solver::solveSphereContacts(Grid &grid, int gridCols, int gridRows,
                                 const std::function<bool(int, int)> &isValidCell) const
{
    if (gridCols <= 0 || gridRows <= 0 || grid.cells.isEmpty())
        return;

    static const QPoint neighborOffsets[] = {
            QPoint(1, 0),
            QPoint(0, 1),
            QPoint(1, 1),
            QPoint(-1, 1)
    };

    auto cellJob = [&grid, gridCols, &isValidCell](unsigned int row, unsigned int col) {
        const int index = static_cast<int>(row * gridCols + col);
        if (index < 0 || index >= grid.cells.size())
            return;

        {
            QMutexLocker locker(grid.locks[index].get());
            auto &cell = grid.cells[index];
            for (int i = 0; i < cell.size(); ++i) {
                for (int j = i + 1; j < cell.size(); ++j) {
                    resolveSpherePair(cell[i], cell[j]);
                }
            }
        }

        auto &cell = grid.cells[index];
        for (const QPoint &offset : neighborOffsets) {
            const int neighborCol = static_cast<int>(col) + offset.x();
            const int neighborRow = static_cast<int>(row) + offset.y();
            if (!isValidCell(neighborCol, neighborRow))
                continue;

            const int neighborIndex = neighborRow * gridCols + neighborCol;
            if (neighborIndex < 0 || neighborIndex >= grid.cells.size())
                continue;

            const int firstIndex  = std::min(index, neighborIndex);
            const int secondIndex = std::max(index, neighborIndex);

            QMutex *firstMutex  = grid.locks[firstIndex].get();
            QMutex *secondMutex = grid.locks[secondIndex].get();
            if (!firstMutex || !secondMutex)
                continue;

            auto processPairs = [&]() {
                auto &neighbor = grid.cells[neighborIndex];
                for (Sphere &a : cell) {
                    for (Sphere &b : neighbor) {
                        resolveSpherePair(a, b);
                    }
                }
            };

            QMutexLocker firstLocker(firstMutex);
            if (secondMutex != firstMutex) {
                QMutexLocker secondLocker(secondMutex);
                processPairs();
            } else {
                processPairs();
            }
        }
    };

    multithreading::forEachCell(grid, cellJob);
}

void Solver::updateVelocities(Grid &grid, float dt) const
{
    if (dt <= 0.f)
        return;

    multithreading::forEachSphere(grid, [dt](Sphere &sphere) {
        QVector2D delta = QVector2D(sphere.position - sphere.prevPosition);
        sphere.velocity = delta / dt;
    });
}

void Solver::applyVelocityDamping(Grid &grid, float dampingFactor) const
{
    multithreading::forEachSphere(grid, [dampingFactor](Sphere &sphere) {
        sphere.velocity *= dampingFactor;
    });
}

void Solver::resolveSpherePair(Sphere &a, Sphere &b)
{
    QVector2D delta = QVector2D(b.position - a.position);
    float dist = delta.length();
    float minDist = a.radius + b.radius;

    if (dist >= minDist)
        return;

    if (dist < 1e-6f) {
        delta = QVector2D(1.f, 0.f);
        dist = 1.f;
    }

    float totalInvMass = a.invMass + b.invMass;
    if (totalInvMass <= 0.f)
        return;

    float penetration = minDist - dist;
    QVector2D normal = delta / dist;
    QVector2D correction = normal * penetration;

    float shareA = a.invMass / totalInvMass;
    float shareB = b.invMass / totalInvMass;

    a.position -= QPointF(correction.x() * shareA, correction.y() * shareA);
    b.position += QPointF(correction.x() * shareB, correction.y() * shareB);
}

Sphere *Solver::findSphereNode(Grid &grid, int groupId, int nodeIndex)
{
    for (QVector<Sphere> &cell : grid.cells) {
        for (Sphere &sphere : cell) {
            if (sphere.groupId == groupId && sphere.nodeIndex == nodeIndex) {
                return &sphere;
            }
        }
    }
    return nullptr;
}