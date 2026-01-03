//
// Created by Tom Favereau on 31/12/2025.
//


#include "multithreading.h"


namespace
{
    /**
     * apply a function on vertical range. on each sphere
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
                    break;
                for (Sphere &sphere : grid.cells[cellIndex]) {
                    task(sphere);
                }
            }
        }
    }

    /**
     * Apply a task verticaly on each cell
     * @param grid
     * @param colBegin
     * @param colEnd
     * @param task
     */
    void process(Grid &grid,
                 int colBegin,
                 int colEnd,
                 const std::function<void (unsigned int, unsigned int)> &task)
    {
        if (!task)
            return;

        colBegin = std::clamp(colBegin, 0, static_cast<int>(grid.gridCols));
        colEnd   = std::clamp(colEnd,   colBegin, static_cast<int>(grid.gridCols));

        for (unsigned int row = 0; row < grid.gridRows; ++row) {
            for (unsigned int col = colBegin; col < colEnd; ++col) task(row, col);
        }
    }

    /**
     * dispatch the thread on different computation zone
     * @tparam Task either a function that act on Sphere or a function that act on cells
     * @param grid
     * @param task
     */
    template <typename Task>
    void dispatch(Grid &grid, Task &&task)
    {
        const int cols = static_cast<int>(grid.gridCols);
        const int maxThreads    = std::max(1, multithreading::maxThreadAllowed());
        const int usableThreads = std::min(maxThreads, cols);

        if (usableThreads <= 1) {
            task(0, cols);
            return;
        }

        const int chunkWidth = std::max(1, (cols + usableThreads - 1) / usableThreads);

        QVector<QFuture<void>> futures;
        futures.reserve(usableThreads);

        for (int colStart = 0; colStart < cols; colStart += chunkWidth) {
            const int colStop = std::min(colStart + chunkWidth, cols);

            futures << QtConcurrent::run(
                    [colStart, colStop, &task]() { task(colStart, colStop); });
        }

        for (QFuture<void> &future : futures) {
            future.waitForFinished();
        }
    }
}

void multithreading::forEachSphere(
        Grid &grid,
        const std::function<void (Sphere &)> &task)
{
    if (!task || grid.isEmpty() || grid.gridCols == 0 || grid.gridRows == 0)
        return;

    dispatch(grid, [&](int colBegin, int colEnd) {
        process(grid, colBegin, colEnd, task);
    });
}

void multithreading::forEachCell(
        Grid &grid,
        const std::function<void (unsigned int, unsigned int)> &task)
{
    if (!task || grid.isEmpty() || grid.gridCols == 0 || grid.gridRows == 0)
        return;

    dispatch(grid, [&](int colBegin, int colEnd) {
        process(grid, colBegin, colEnd, task);
    });
}

int multithreading::maxThreadAllowed()
{
    return QThreadPool::globalInstance()->maxThreadCount();
}