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
 * this class represent a physical objet
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

    [[maybe_unused]] [[nodiscard]] float mass() const //infinity in case we want to do static sphere. but now that we already have those it may be unnecessary
    {
        return invMass <= 0.f ? std::numeric_limits<float>::infinity() : 1.f / invMass;
    }
};

class Sphere : public PhysicalBody
{
public:
    Sphere() = default;
    explicit Sphere(float radiusValue) // forbid conversion
            : radius(radiusValue) {}

    float radius = 30.f;

    int groupId = -1;    // -1 = Independant sphere
    int nodeIndex = -1;  // index in the square 0, 1, 2, 3
};

#endif // PHYSICALBODY_H
