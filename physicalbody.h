//
// Created by Tom Favereau on 28/12/2025.
//

#ifndef PHYSICALBODY_H
#define PHYSICALBODY_H

#include <QPointF>
#include <QVector2D>
#include <QColor>
#include <limits>


/**
 * this class represent
 */
class PhysicalBody
{
public:
    PhysicalBody() = default;
    virtual ~PhysicalBody() = default;

    QPointF position = QPointF(0.0, 0.0);
    QPointF prevPosition = QPointF(0.0, 0.0);
    QVector2D velocity = QVector2D(0.f, 0.f);
    float invMass = 1.f;
    QColor color = Qt::blue;

    void setMass(float mass)
    {
        if (mass <= 0.f) {
            invMass = 0.f;
        } else {
            invMass = 1.f / mass;
        }
    }

    float mass() const
    {
        return invMass <= 0.f ? std::numeric_limits<float>::infinity() : 1.f / invMass;
    }
};

class Sphere : public PhysicalBody
{
public:
    Sphere() = default;
    explicit Sphere(float radiusValue)
            : radius(radiusValue) {}

    float radius = 30.f;
};

#endif // PHYSICALBODY_H
