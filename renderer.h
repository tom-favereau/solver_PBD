//
// Created by Tom Favereau on 03/01/2026.
//

#ifndef SOLVER_RENDERER_H
#define SOLVER_RENDERER_H

#include "context.h"
#include "grid.h"
#include "constraints.h"
#include "physicalbody.h"

#include <QPainter>
#include <QPen>
#include <QBrush>
#include <memory>


namespace renderer
{
    /**
     * render the simulation each iteration
     * @param painter
     * @param context owner of the grid
     */
    void render(QPainter &painter, const Context &context) ;
};


#endif //SOLVER_RENDERER_H
