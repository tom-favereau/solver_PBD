#ifndef DRAWAREA_H
#define DRAWAREA_H

#include <QWidget>
#include <QVector>
#include <QTimer>
#include <QElapsedTimer>
#include <memory>
#include <QKeyEvent>

#include "physicalbody.h"
#include "constraints.h"
#include "multithreading.h"

class DrawArea : public QWidget
{
Q_OBJECT // I don't realy know what it does but don't touch
public:
    explicit DrawArea(QWidget *parent = nullptr);

protected:
    /**
     * Add a sphere (radius 30) when mouse is clicked
     * @param event
     */
    void mousePressEvent(QMouseEvent *event) override;

    /**
     *
     * @param event
     */
    void paintEvent(QPaintEvent *event) override;

    /**
     * rebuild the grid and the constaint when the window is rezized
     * @param event
     */
    void resizeEvent(QResizeEvent *event) override;

    /**
     * add small sphere (radius 5) which initial speed is cosinus of time.
     * @param event
     */
    void keyPressEvent(QKeyEvent *event) override;

    /**
     * stop adding sphere
     * @param event
     */
    void keyReleaseEvent(QKeyEvent *event) override;



private slots: // I don't really know what it does but don't touch
    /**
     * handle the main loop of simulation
     */
    void animate();

    /**
     * handle the generation of small sphere : keyPressEvent
     */
    void emitCenterSphere();


private:

    /**
     * Initialize the grid a vector of size hg*wg where hg = height/targetcellsize
     * @param initialSize
     */
    void initializeGrid(const QSize &initialSize);

    /**
     * Rebuild the grid according to a new size if necessary replacing the sphere at there position
     * @param newSize
     */
    void rebuildGrid(const QSize &newSize);

    /**
     * rebuild the static constraint accroding to the new size in case if resizing the window
     */
    void rebuildBoundaryConstraints();

    /**
     * Insert a sphere in grid, the position is a member of sphere
     * @param sphere
     */
    void insertSphere(const Sphere &sphere);

    /**
     * Actualize the grid after one step of simulation
     */
    void rehashGrid();
    [[nodiscard]] int cellIndexFor(const QPointF &position) const;
    [[nodiscard]] QPoint cellCoords(int cellIndex) const;
    [[nodiscard]] bool isValidCell(int col, int row) const;


    QTimer emissionTimer;// used with binding "e"
    bool isEmitting = false; // used with binding "e"

    [[nodiscard]] bool isCenterCellEmpty() const;


    /**
     * Apply the external forces and compute the new position
     * @param dt
     */
    void integrateBodies(float dt);

    /**
     * resolve static constraint with method project from static Constraint
     */
    void satisfyStaticConstraints();

    /**
     * resolve sphere contact with method resolveSpherePair
     */
    void solveSphereContacts();

    /**
     * Correct the penetration almost the same way we resolve static sphere constraintexcept that we take mass into acount
     * @param a
     * @param b
     */
    static void resolveSpherePair(Sphere &a, Sphere &b);

    /**
     * Recompute velicities accoding to position and previous position
     * @param dt
     */
    void updateVelocities(float dt);

    /**
     * Apply damping
     * @param Dampingfactor
     */
    void applyVelocityDamping(float Dampingfactor);

    // Cluster management ----------------- this should derive Physical body in some way
    struct SpringLink
    {
        int groupId = -1;
        int aNode = -1;
        int bNode = -1;
        float restLength = 0.f;
        float stiffness = 0.9f;
    };

    Sphere *findSphereNode(int groupId, int nodeIndex);
    const Sphere *findSphereNode(int groupId, int nodeIndex) const;

    void satisfySpringConstraints();
    void createSpringCluster(const QPointF &center);

    QVector<SpringLink> springLinks;
    int nextGroupId = 0;
    //------------------------------------------

    QVector<QVector<Sphere>> grid;
    QVector<std::shared_ptr<StaticConstraint>> staticConstraints;

    int gridCols = 1;
    int gridRows = 1;
    float cellWidth = 1.f;
    float cellHeight = 1.f;
    const float targetCellSize = 200;//120.f; // I don't know which value to chose

    QTimer timer;
    QElapsedTimer clock;
    qint64 lastTick = 0;
};


#endif // DRAWAREA_H