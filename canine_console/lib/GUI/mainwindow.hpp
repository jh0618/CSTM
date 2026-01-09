#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "SharedMemory.hpp"
#include "qcustomplot.h"
#include <iostream>

QT_BEGIN_NAMESPACE
namespace Ui
{
    class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
Q_OBJECT

public:
    MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

private slots:
    void DisplayUpdate();
    void DeveloperDisplayUpdate();
    void clearGraph(QCustomPlot* QCP);
    void on_BT_GRAPH_RESET_clicked();
    void on_BT_GRAPH_DOWN_clicked();
    void on_BT_GRAPH_UP_clicked();
    void on_BT_GRAPH_PAUSE_GRAPH_clicked();
    void on_BT_ROBOT_START_clicked();
    void on_BT_E_STOP_clicked();
    void on_BT_STAND_UP_clicked();
    void on_BT_SIT_DOWN_clicked();
    void on_BT_TROT_OVERLAP_clicked();
    void on_BT_TROT_SLOW_clicked();
    void on_BT_TROT_FAST_clicked();
    void on_BT_TROT_STOP_clicked();
    void on_BT_ROBOT_RESTART_clicked();
    void on_BT_IP_ADDRESS_clicked();

private:
    void scaleScreen();
    void checkPrevData();
    void GuiInitialize();
    void GraphInitialize();
    void addGraphData();

private:
    Ui::MainWindow* ui;
    SharedMemory* sharedMemory;
    QTimer* displayTimer;
    QTimer* graphTimer;
    bool isFullScreenMode;
    double graphOffset;

    bool mIsUpdateGraph = true;
    bool mPrevDeveloperModeChecked = false;
    bool mNewGUIinputSignal = false;
    int mPrevFSMState = FSM_INITIAL;
    bool mPrevTCPConnected = false;
    bool mPrevMotorOffsetSet = false;
    bool mPrevMotorConnected = false;
    bool mPrevMotorControlStart = false;
};

#endif // MAINWINDOW_H
