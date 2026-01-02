//
// Created by Tom Favereau on 31/12/2025.
//

#ifndef TD9_MULTITHREADING_H
#define TD9_MULTITHREADING_H

#include <functional>
#include <QVector>
#include <QtConcurrent/QtConcurrentMap>
#include <QThreadPool>

#include "physicalbody.h"


class multithreading
{
public:


    /**
     * for each sphere apply a procedure
     * @param grid reference on grid
     * @param task procdure applied
     * @param multithreaded is multithreading active or not
     */
    static void forEachSphere(QVector<QVector<Sphere>> &grid,
                              const std::function<void (Sphere &)> &task,
                              bool multithreaded = true);

    /**
     * Return the number of thread needed
     */
    static int idealThreadCount();
};


#endif //TD9_MULTITHREADING_H
