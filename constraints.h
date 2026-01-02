#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H

#include "physicalbody.h"
#include <QVector2D>
#include <QPointF>


/**
 * A static constraint is a generic constraint it contains a method projects that solve a collision with a sphere
 */
class StaticConstraint
{
public:
    virtual ~StaticConstraint() = default;

    /**
     * Resolve a colision with a sphere
     * @param sphere
     */
    virtual void project(Sphere &sphere) const = 0;
};


/**
 * A plane constraint is define with by the equation : dot(X, normal) = distance
 */
class PlaneConstraint : public StaticConstraint
{
public:
    PlaneConstraint() = default;

    PlaneConstraint(const QVector2D &normal, float distance);

    //void set(const QVector2D &normal, float distance);

    /**
     * Compute the signed distance between the sphere and the plane and correct accordingly
     * @param sphere
     */
    void project(Sphere &sphere) const override;

private:
    QVector2D m_normal = QVector2D(0.f, 1.f);
    float m_distance = 0.f;
};


/**
 * A sphere constraint is define with a center and a radius. It is handle the same way as sphere colision.
 */
class SphereConstraint : public StaticConstraint
{
public:
    SphereConstraint() = default;

    SphereConstraint(const QPointF &center, float radius);

    /**
     * compute the penetration and resolve acordingly
     * @param sphere
     */
    void project(Sphere &sphere) const override;

    [[nodiscard]] const QPointF &center() const { return m_center; }
    [[nodiscard]] float radius() const { return m_radius; }

private:
    QPointF m_center = QPointF(0.0, 0.0);
    float m_radius = 10.f;
};

class BowlConstraint : public StaticConstraint
{
public:
    BowlConstraint() = default;

    BowlConstraint(const QPointF &center, float radius);

    /**
     * Compute the penetration and resolve acrodingly
     * @param sphere
     */
    void project(Sphere &sphere) const override;

    [[nodiscard]] const QPointF &center() const { return m_center; }
    [[nodiscard]] float radius() const { return m_radius; }

private:
    QPointF m_center = QPointF(0.0, 0.0);
    float m_radius = 100.f;
};

#endif // CONSTRAINTS_H