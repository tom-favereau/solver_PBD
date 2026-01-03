#include "drawarea.h"

#include <QMouseEvent>
#include <QPainter>
#include <QThreadPool>

DrawArea::DrawArea(QWidget *parent, unsigned int hearts)
        : QWidget(parent)
{
    setFocusPolicy(Qt::StrongFocus);
    setStyleSheet("background: white;");

    if (hearts != 0) {
        QThreadPool::globalInstance()->setMaxThreadCount(static_cast<int>(hearts));
    }

    QSize initialSize = size();
    if (initialSize.isEmpty()) {
        initialSize = QSize(800, 600);
        resize(initialSize);
    }

    context.initialize(initialSize);

    connect(&timer, &QTimer::timeout, this, &DrawArea::animate);
    timer.start(16);

    emissionTimer.setInterval(120);
    connect(&emissionTimer, &QTimer::timeout, this, &DrawArea::emitCenterSphere);

    clock.start();
    lastTick = clock.nsecsElapsed();
}

void DrawArea::mousePressEvent(QMouseEvent *event)
{
    context.addUserSphere(event->pos());
    update();
}

void DrawArea::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    renderer::render(painter, context);
}

void DrawArea::resizeEvent(QResizeEvent *event)
{
    context.resizeScene(event->size());
    QWidget::resizeEvent(event);
}

void DrawArea::keyPressEvent(QKeyEvent *event)
{
    if (event->isAutoRepeat()) {
        QWidget::keyPressEvent(event);
        return;
    }

    if (event->key() == Qt::Key_E) {
        if (!isEmitting) {
            isEmitting = true;
            emitCenterSphere();
            emissionTimer.start();
        }
        event->accept();
        return;
    }

    if (event->key() == Qt::Key_C) {
        context.createSpringCluster(context.sceneCenter());
        event->accept();
        update();
        return;
    }

    QWidget::keyPressEvent(event);
}

void DrawArea::keyReleaseEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_E && !event->isAutoRepeat()) {
        //if (isEmitting) {
        //    emissionTimer.stop();
        //    isEmitting = false;
        //}
        //event->accept();
        return;
    }
    QWidget::keyReleaseEvent(event);
}

void DrawArea::animate()
{
    qint64 now = clock.nsecsElapsed();
    float frameDt = static_cast<float>((now - lastTick)) * 1e-9f;
    lastTick = now;

    if (frameDt <= 0.f)
        frameDt = 1.f / 60.f;

    frameDt = std::clamp(frameDt, 1.f / 240.f, 1.f / 20.f);

    context.step(frameDt);
    update();
}

void DrawArea::emitCenterSphere()
{
    const float t = static_cast<float>(clock.nsecsElapsed()) * 1e-9f;
    context.emitCenterSphere(t);
    update();
}