#ifndef DRAWAREA_H
#define DRAWAREA_H

#include <QWidget>
#include <QTimer>
#include <QElapsedTimer>

#include "context.h"
#include "renderer.h"

class DrawArea : public QWidget
{
Q_OBJECT
public:
    /**
     *
     * @param parent Qwidget
     * @param hearts number of heart that the program is allowed to use. 0 = maximum
     */
    explicit DrawArea(QWidget *parent = nullptr, unsigned int hearts = 0);

protected:
    /**
     * handle click event -> add a sphere at the posisition the user click
     * @param event
     */
    void mousePressEvent(QMouseEvent *event) override;

    /**
     * call the renderer. This method is called by Qt when show is used and at each loop of animate
     * @param event
     */
    void paintEvent(QPaintEvent *event) override;

    /**
     * rebuild the scene and the context acording to a new size for the window
     * @param event
     */
    void resizeEvent(QResizeEvent *event) override;

    /**
     * handle key press event :
     * c = square in the center
     * e = emit small sphere in the center
     * @param event
     */
    void keyPressEvent(QKeyEvent *event) override;

    /**
     * stop the emision of sphere
     * @param event
     */
    void keyReleaseEvent(QKeyEvent *event) override;

private slots:
    /**
     * main loop of the simulation
     */
    void animate();

    /**
     * loop for emiting small sphere
     */
    void emitCenterSphere();

private:
    Context context;

    QTimer timer;
    QTimer emissionTimer;
    QElapsedTimer clock;

    qint64 lastTick = 0;
    bool isEmitting = false;
};

#endif // DRAWAREA_H