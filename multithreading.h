//
// Created by Tom Favereau on 31/12/2025.
//

#ifndef TD9_MULTITHREADING_H
#define TD9_MULTITHREADING_H

#include <functional>
#include <QVector>
#include <QtConcurrent/QtConcurrentMap>
#include <QtConcurrent/QtConcurrentRun>
#include <QThreadPool>

#include "physicalbody.h"
#include "grid.h"


namespace multithreading
{
    /**
     * for each sphere apply a procedure
     * @param grid reference on grid
     * @param task procdure applied
     */
    void forEachSphere(Grid &grid, const std::function<void (Sphere &)> &task);

    /**
     * for each cell apply a procedure
     * @param grid
     * @param task
     */
    void forEachCell(Grid &grid, const std::function<void (unsigned int, unsigned int)> &task);

    /**
     * Return the number of thread allowed
     */
    int maxThreadAllowed();
};


#endif //TD9_MULTITHREADING_H
