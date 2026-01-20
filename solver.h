//
// Created by Tom Favereau on 03/01/2026.
//

#ifndef SOLVER_SOLVER_H
#define SOLVER_SOLVER_H

#include "grid.h"
#include "physicalbody.h"
#include "constraints.h"
#include "springlink.h"
#include "multithreading.h"

#include <QMutexLocker>
#include <QPoint>
#include <QVector2D>
#include <QVector>
#include <functional>
#include <memory>

/**
 * Position based dynamics solver
 */
namespace solver
{


    /**
     * Apply the external forces and compute the new position
     * @param dt
     */
    void integrateBodies(Grid &grid, float dt) ;

    /**
     * resolve static constraint with method project from static Constraint
     */
    void satisfyStaticConstraints(Grid &grid, const QVector<std::shared_ptr<StaticConstraint>> &constraints) ;

    /**
     * resolve spring constraint cluster by cluster
     * @param grid
     * @param springLinks
     */
    void satisfySpringConstraints(Grid &grid, QVector<SpringLink> &springLinks, unsigned int subSteps);

    /**
    * resolve sphere contact with method resolveSpherePair
    * this funcrion call multithreading lockers are assure with QMutex
    */
    void solveSphereContacts(Grid &grid,
                             int gridCols,
                             int gridRows,
                             const std::function<bool(int, int)> &isValidCell) ;

    /**
     * Recompute velicities accoding to position and previous position acording to position based dynamics
     * @param dt
     */
    void updateVelocities(Grid &grid, float dt) ;

    /**
     * Apply damping
     * @param Dampingfactor
     */
    void applyVelocityDamping(Grid &grid, float dampingFactor) ;



}


#endif //SOLVER_SOLVER_H
