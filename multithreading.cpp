//
// Created by Tom Favereau on 31/12/2025.
//


#include "multithreading.h"


namespace
{
    /**
     * apply a function on vertical range.
     * @param grid
     * @param colBegin
     * @param colEnd
     * @param task
     */
    void process(Grid &grid,
                            int colBegin,
                            int colEnd,
                            const std::function<void (Sphere &)> &task)
    {
        if (!task || grid.gridCols == 0 || grid.gridRows == 0)
            return;

        colBegin = std::clamp(colBegin, 0, static_cast<int>(grid.gridCols));
        colEnd   = std::clamp(colEnd,   colBegin, static_cast<int>(grid.gridCols));

        for (unsigned int row = 0; row < grid.gridRows; ++row) {
            const int baseIndex = static_cast<int>(row * grid.gridCols);
            for (int col = colBegin; col < colEnd; ++col) {
                const int cellIndex = baseIndex + col;
                if (cellIndex >= grid.size())
                    break; // sécurité (au cas où grid.size() < rows*cols)
                for (Sphere &sphere : grid.cells[cellIndex]) {
                    task(sphere);
                }
            }
        }
    }
}

void multithreading::forEachSphere(Grid &grid,
                                   const std::function<void (Sphere &)> &task)
{
    if (!task || grid.isEmpty() || grid.gridCols == 0 || grid.gridRows == 0) {
        return;
    }

    const int maxThreads = std::max(1, maxThreadAllowed());
    const int usableThreads = std::min(maxThreads, static_cast<int>(grid.gridCols));

    if (usableThreads <= 1) {
        process(grid, 0, static_cast<int>(grid.gridCols), task);
        return;
    }

    const int chunkWidth = std::max(
            1, (static_cast<int>(grid.gridCols) + usableThreads - 1) / usableThreads);

    QList<QFuture<void>> futures;
    futures.reserve(usableThreads);

    for (int colStart = 0; colStart < static_cast<int>(grid.gridCols); colStart += chunkWidth) {
        const int colStop = std::min(colStart + chunkWidth,
                                     static_cast<int>(grid.gridCols));

        futures << QtConcurrent::run([&grid, colStart, colStop, &task]() {
            process(grid, colStart, colStop, task);
        });
    }

    for (QFuture<void> &future : futures) {
        future.waitForFinished();
    }
}

int multithreading::maxThreadAllowed()
{
    return QThreadPool::globalInstance()->maxThreadCount();
}