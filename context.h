//
// Created by Tom Favereau on 03/01/2026.
//

#ifndef TD9_CONTEXT_H
#define TD9_CONTEXT_H


#include <QVector>
#include <QSize>
#include <QPointF>
#include <memory>
#include <QRandomGenerator>
#include <QVector2D>
#include <algorithm>
#include <cmath>
#include <utility>

#include "grid.h"
#include "constraints.h"
#include "physicalbody.h"
#include "springlink.h"
#include "solver.h"

/**
 * Own the grid and the element of the simulation.
 */
class Context {
public:
    Context() = default;

    /**
     * initialize the grid and the static constraint acording to the initial size
     * @param initialSize
     */
    void initialize(const QSize &initialSize);

    /**
     * rebuild the grid to its new size and rebuild the constraint
     * @param newSize
     */
    void resizeScene(const QSize &newSize);

    /**
     * Solve the constraint with 4 iteration of (static -> spring -> sphere) and then update velocities
     * @param frameDt
     */
    void step(float frameDt);

    /**
     * add a sphere in the grid where it is clicked
     * @param position
     */
    void addUserSphere(const QPointF &position);

    /**
     * emit small sphere from the center when "e" is pressed
     * @param timeSeconds
     */
    void emitCenterSphere(float timeSeconds);

    /**
     * create a square cluster when "c" is pressed
     * @param center
     */
    void createSpringCluster(const QPointF &center);

    [[nodiscard]] bool isCenterCellEmpty() const;
    [[nodiscard]] QPointF sceneCenter() const;
    [[nodiscard]] QSize sceneSize() const { return sceneSize_; }

    [[nodiscard]] const Grid &grid() const { return grid_; }
    Grid &grid() { return grid_; }

    [[nodiscard]] const QVector<std::shared_ptr<StaticConstraint>> &constraints() const { return staticConstraints; }

private:
    /**
     * initialize the grid
     * @param size
     */
    void initializeGrid(const QSize &size);

    /**
     * rebuild the grid in case of a rize event, replacing the sphere
     * @param size
     */
    void rebuildGrid(const QSize &size);

    /**
     * rebuild the constraint according to the new size of the scene
     */
    void rebuildStaticConstraints();

    /**
     * insert a sphere in the grid
     * @param sphere
     */
    void insertSphere(const Sphere &sphere);

    /**
     * update the grid according to the new position of the sphere
     */
    void updateGrid();

    [[nodiscard]] int cellIndexFor(const QPointF &position) const;
    [[nodiscard]] bool isValidCell(int col, int row) const;


    Grid grid_;
    QVector<std::shared_ptr<StaticConstraint>> staticConstraints;
    QVector<SpringLink> springLinks;

    int nextGroupId   = 0;
    int gridCols      = 1;
    int gridRows      = 1;
    float cellWidth   = 1.f;
    float cellHeight  = 1.f;
    float targetCellSize = 200.f;

    QSize sceneSize_ {800, 600};
};


#endif //TD9_CONTEXT_H
