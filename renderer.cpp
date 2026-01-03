//
// Created by Tom Favereau on 03/01/2026.
//

#include "renderer.h"


namespace
{
    constexpr QColor kConstraintStroke(0, 102, 255, 200);
    constexpr QColor kConstraintFill(0, 102, 255, 40);

}

void renderer::render(QPainter &painter, const Context &context)
{
    const Grid &grid = context.grid();

    for (const QVector<Sphere> &cell : grid.cells) {
        for (const Sphere &sphere : cell) {
            QPen pen(sphere.color, 3);
            painter.setPen(pen);
            painter.setBrush(QBrush(sphere.color));
            painter.drawEllipse(sphere.position, sphere.radius, sphere.radius);
        }
    }

    QPen constraintPen(kConstraintStroke, 2);
    constraintPen.setStyle(Qt::DashLine);
    constraintPen.setCosmetic(true);//doesnt resize the line if the window is risize.
    painter.setPen(constraintPen);
    painter.setBrush(QBrush(kConstraintFill));

    for (const auto &constraint : context.constraints()) {
        if (!constraint)
            continue;

        if (auto sphereConstraint = std::dynamic_pointer_cast<SphereConstraint>(constraint)) {
            painter.drawEllipse(sphereConstraint->center(), sphereConstraint->radius(), sphereConstraint->radius());
        } else if (auto bowlConstraint = std::dynamic_pointer_cast<BowlConstraint>(constraint)) {
            painter.drawEllipse(bowlConstraint->center(), bowlConstraint->radius(), bowlConstraint->radius());

        }
    }
}