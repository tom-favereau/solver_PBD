//
// Created by Tom Favereau on 02/01/2026.
//

#ifndef TD9_GRID_H
#define TD9_GRID_H

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

    [[nodiscard]] int size()  const { return cells.size(); }

    QVector<QVector<Sphere>> cells;
    unsigned int gridRows;
    unsigned int gridCols;
    //const float targetCellSize = 200;

    [[nodiscard]] bool isEmpty() const { return cells.isEmpty(); }
};

#endif //TD9_GRID_H
