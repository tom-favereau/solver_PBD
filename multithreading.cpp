//
// Created by Tom Favereau on 31/12/2025.
//

#include "multithreading.h"
#include <iostream>


namespace
{
    struct CellFunctor
    {
        std::function<void (Sphere &)> task;

        void operator()(QVector<Sphere> &cell) const
        {
            for (Sphere &sphere : cell) {
                task(sphere);
            }
        }
    };
}


void multithreading::forEachSphere(QVector<QVector<Sphere>> &grid,
                                           const std::function<void (Sphere &)> &task,
                                           bool multithreaded)
{
    if (!task) {
        return;
    }

    const bool canParallelize = multithreaded &&
                                grid.size() > 1 &&
                                idealThreadCount() > 1;

    if (!canParallelize) {
        for (QVector<Sphere> &cell : grid) {
            for (Sphere &sphere : cell) {
                task(sphere);
            }
        }
        return;
    }

    CellFunctor functor{task};
    QtConcurrent::blockingMap(grid, functor);
}

int multithreading::idealThreadCount()
{
    return QThreadPool::globalInstance()->maxThreadCount();
}