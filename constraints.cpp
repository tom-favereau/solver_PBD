#include "constraints.h"
#include <QtMath>

PlaneConstraint::PlaneConstraint(const QVector2D &normal, float distance)
{
    QVector2D n = normal;
    if (n.isNull()) {
        n = QVector2D(0.f, 1.f); // just a non zero vector to not raise an ecept. maybe we should
    } else {
        n.normalize();
    }
    m_normal = n;
    m_distance = distance;
}

void PlaneConstraint::project(Sphere &sphere) const
{
    if (sphere.invMass <= 0.f)
        return;

    QVector2D center = QVector2D(sphere.position);
    float signedDistance = QVector2D::dotProduct(m_normal, center) - m_distance - sphere.radius;

    if (signedDistance < 0.f) {
        QVector2D correction = -signedDistance * m_normal;
        sphere.position += QPointF(correction.x(), correction.y());
    }
}

void SphereConstraint::set(const QPointF &center, float radius)
{
    m_center = center;
    m_radius = qMax(0.f, radius);
}

void SphereConstraint::project(Sphere &sphere) const
{
    if (sphere.invMass <= 0.f)
        return;

    QVector2D delta = QVector2D(sphere.position - m_center);
    float dist = delta.length();
    float minDist = m_radius + sphere.radius;

    if (dist >= minDist)
        return;

    if (dist < 1e-5f) {
        delta = QVector2D(1.f, 0.f);
        dist = 1.f;
    }

    QVector2D normal = delta / dist;
    float penetration = minDist - dist;
    QVector2D correction = normal * penetration;

    sphere.position += QPointF(correction.x(), correction.y());
}

void BowlConstraint::set(const QPointF &center, float radius)
{
    m_center = center;
    m_radius = qMax(0.f, radius);
}

void BowlConstraint::project(Sphere &sphere) const
{
    if (sphere.invMass <= 0.f || m_radius <= 0.f)
        return;

    QVector2D delta = QVector2D(sphere.position - m_center);
    float dist = delta.length();

    float maxDist = qMax(0.f, m_radius - sphere.radius);
    if (dist <= maxDist)
        return;

    if (dist < 1e-5f) {
        // Si la sphère est exactement au centre, on l’éloigne un peu
        delta = QVector2D(0.f, -1.f);
        dist = 1.f;
    }

    QVector2D normal = delta / dist;
    float penetration = dist - maxDist;
    QVector2D correction = normal * penetration;

    // On la ramène vers l’intérieur de la cuvette
    sphere.position -= QPointF(correction.x(), correction.y());
}