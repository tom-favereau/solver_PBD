//
// Created by Tom Favereau on 02/01/2026.
//

#ifndef TD9_GRID_H
#define TD9_GRID_H

#include <QMutex>
#include "physicalbody.h"


/**
 * Grid structure
 * maybe we should look at gmsh or something similar to be able to refine the mesh where it is needed
 * because some area has less ball than others. Or kdtree but it scars me :)
 */
class Grid {

public:
    Grid() : gridRows(0), gridCols(0) {}

    Grid(unsigned int rows, unsigned int cols)
            : cells(static_cast<int>(rows * cols)),
              gridRows(rows),
              gridCols(cols) {}

    ~Grid() = default;


    void resize(unsigned int rows, unsigned int cols) {
        gridRows = rows;
        gridCols = cols;
        const int total = static_cast<int>(rows * cols);

        cells.resize(total);
        locks.resize(total);
        for (int i = 0; i < total; ++i) {
            if (!locks[i])
                locks[i].reset(new QMutex);
        }
    }


    [[nodiscard]] int size()  const { return cells.size(); }
    [[nodiscard]] bool isEmpty() const { return cells.isEmpty(); }

    QVector<QVector<Sphere>> cells;
    std::vector<std::unique_ptr<QMutex>> locks; // we don't use QVector because it does not support Qmutex to have copy constructor deleted
    unsigned int gridRows;
    unsigned int gridCols;
    //const float targetCellSize = 200;


};

#endif //TD9_GRID_H
