#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include "drawarea.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    auto *area = new DrawArea(this);
    setCentralWidget(area);
}

MainWindow::~MainWindow()
{
    delete ui;
}
