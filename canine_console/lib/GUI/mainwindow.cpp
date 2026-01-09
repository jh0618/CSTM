#include "mainwindow.hpp"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , isFullScreenMode(true)
    , graphOffset(5.0)
{
    ui->setupUi(this);
    sharedMemory = SharedMemory::getInstance();
    GuiInitialize();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::GuiInitialize()
{
//    InitTable(ui->TB_MOTOR);
    QMainWindow::setWindowIcon(QIcon(QString(GUI_RSC_DIR) + "COMBINED_IMAGE_RB_ON_CAMEL.png"));
    QString logoPath = QString(GUI_RSC_DIR) + "CAMEL_logo.png";
    QPixmap logoPixmap(logoPath);
    ui->LB_IMG_CAMEL->setPixmap(logoPixmap.scaled(ui->LB_IMG_CAMEL->size(), Qt::KeepAspectRatio));
    ui->LB_IMG_CAMEL->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
    connect(ui->BT_QUIT, &QPushButton::clicked, [this]()
    { qApp->quit(); });
    connect(ui->BT_SCALE_UP, &QPushButton::clicked, this, &MainWindow::scaleScreen);
    connect(ui->BT_MINIMIZE, &QPushButton::clicked, [this]()
    { this->showMinimized(); });
    GraphInitialize();

    displayTimer = new QTimer();
    connect(displayTimer, SIGNAL(timeout()), this, SLOT(DisplayUpdate()));
    displayTimer->start(20);
    graphTimer = new QTimer();
    connect(graphTimer, SIGNAL(timeout()), this, SLOT(DeveloperDisplayUpdate()));
    graphTimer->start(20);
    ui->TE_IP_ADDRESS->setText(QString(QString::fromStdString(INIT_IP)));
    ui->BT_ROBOT_START->setDisabled(true);
    ui->LE_ROBOT_STATE->setText("DISCONNECTED");
    ui->LE_CONNECT_1->setStyleSheet("background-color:red");
    ui->LE_CONNECT_2->setStyleSheet("background-color:red");
    ui->BT_ROBOT_START->setDisabled(true);
    ui->BT_STAND_UP->setDisabled(true);
    ui->BT_E_STOP->setDisabled(true);
    ui->BT_SIT_DOWN->setDisabled(true);
    ui->BT_TROT_SLOW->setDisabled(true);
    ui->BT_TROT_FAST->setDisabled(true);
    ui->BT_TROT_OVERLAP->setDisabled(true);
    ui->BT_TROT_STOP->setDisabled(true);
    ui->BT_ROBOT_START->setDisabled(true);
    ui->BT_ROBOT_RESTART->setDisabled(true);
    ui->LE_TROT_OVERLAP->setStyleSheet("background-color:pink");
    ui->LE_TROT_SLOW->setStyleSheet("background-color:pink");
    ui->LE_TROT_FAST->setStyleSheet("background-color:pink");
    ui->LE_TROT_STOP->setStyleSheet("background-color:pink");
    ui->CB_DEVELOPER_MODE->setCheckState(Qt::CheckState::Checked);
    ui->GB_GRAPH_OPTION->hide();
}

void MainWindow::scaleScreen()
{
    if (isFullScreenMode)
    {
        isFullScreenMode = false;
        this->showNormal();
    }
    else
    {
        isFullScreenMode = true;
        this->showFullScreen();
    }
}

void MainWindow::GraphInitialize()
{
    QPen myPen, desiredPen, dotPen, filterPen;
    myPen.setWidthF(1);
    filterPen.setStyle(Qt::DotLine);
    filterPen.setWidth(1);
    dotPen.setStyle(Qt::DotLine);
    dotPen.setWidth(20);
    dotPen.setWidthF(2);
    dotPen.setColor(Qt::gray);

    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%m:%s");

    ui->QCP_BODY_POS_X->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_BODY_POS_Y->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_BODY_POS_Z->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_BODY_VEL_X->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_BODY_VEL_Y->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_BODY_VEL_Z->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);

    ui->QCP_BODY_ANG_POS_X->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_BODY_ANG_VEL_X->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_BODY_ANG_POS_Y->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_BODY_ANG_VEL_Y->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_BODY_ANG_POS_Z->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_BODY_ANG_VEL_Z->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);

    ui->QCP_MT_POS_FLHR->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_MT_POS_FLHP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_MT_POS_FLKP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_MT_POS_FRHR->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_MT_POS_FRHP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_MT_POS_FRKP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_MT_POS_HLHR->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_MT_POS_HLHP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_MT_POS_HLKP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_MT_POS_HRHR->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_MT_POS_HRHP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_MT_POS_HRKP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);

    ui->QCP_MT_VEL_FLHR->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_MT_VEL_FLHP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_MT_VEL_FLKP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_MT_VEL_FRHR->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_MT_VEL_FRHP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_MT_VEL_FRKP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_MT_VEL_HLHR->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_MT_VEL_HLHP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_MT_VEL_HLKP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_MT_VEL_HRHR->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_MT_VEL_HRHP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_MT_VEL_HRKP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);

    ui->QCP_MT_TAU_FLHR->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_MT_TAU_FLHP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_MT_TAU_FLKP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_MT_TAU_FRHR->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_MT_TAU_FRHP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_MT_TAU_FRKP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_MT_TAU_HLHR->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_MT_TAU_HLHP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_MT_TAU_HLKP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_MT_TAU_HRHR->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_MT_TAU_HRHP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_MT_TAU_HRKP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_CONTACT_FL->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_CONTACT_FR->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_CONTACT_HL->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->QCP_CONTACT_HR->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);

    myPen.setColor(Qt::blue);
    desiredPen.setColor(Qt::red);
    int fontsize = 7;
    myPen.setWidthF(1.0);
    desiredPen.setWidthF(1.0);
    ui->QCP_BODY_POS_X->legend->setVisible(true);
    ui->QCP_BODY_POS_X->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_BODY_POS_X->addGraph();
    ui->QCP_BODY_POS_X->graph(0)->setPen(myPen);
    ui->QCP_BODY_POS_X->graph(0)->setName("Current Px");
    ui->QCP_BODY_POS_X->addGraph();
    ui->QCP_BODY_POS_X->graph(1)->setPen(dotPen);
    ui->QCP_BODY_POS_X->graph(1)->setName("Desired Px");
    ui->QCP_BODY_POS_X->addGraph();
    ui->QCP_BODY_POS_X->graph(2)->setPen(desiredPen);
    ui->QCP_BODY_POS_X->graph(2)->setName("base line Px");

    ui->QCP_BODY_POS_Y->legend->setVisible(true);
    ui->QCP_BODY_POS_Y->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_BODY_POS_Y->addGraph();
    ui->QCP_BODY_POS_Y->graph(0)->setPen(myPen);
    ui->QCP_BODY_POS_Y->graph(0)->setName("Current Py");
    ui->QCP_BODY_POS_Y->addGraph();
    ui->QCP_BODY_POS_Y->graph(1)->setPen(dotPen);
    ui->QCP_BODY_POS_Y->graph(1)->setName("Desired Py");
    ui->QCP_BODY_POS_Y->addGraph();
    ui->QCP_BODY_POS_Y->graph(2)->setPen(desiredPen);
    ui->QCP_BODY_POS_Y->graph(2)->setName("base line Py");

    ui->QCP_BODY_POS_Z->legend->setVisible(true);
    ui->QCP_BODY_POS_Z->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_BODY_POS_Z->addGraph();
    ui->QCP_BODY_POS_Z->graph(0)->setPen(myPen);
    ui->QCP_BODY_POS_Z->graph(0)->setName("Current Pz");
    ui->QCP_BODY_POS_Z->addGraph();
    ui->QCP_BODY_POS_Z->graph(1)->setPen(dotPen);
    ui->QCP_BODY_POS_Z->graph(1)->setName("Desired Pz");
    ui->QCP_BODY_POS_Z->addGraph();
    ui->QCP_BODY_POS_Z->graph(2)->setPen(desiredPen);
    ui->QCP_BODY_POS_Z->graph(2)->setName("base line Pz");

    ui->QCP_BODY_VEL_X->legend->setVisible(true);
    ui->QCP_BODY_VEL_X->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_BODY_VEL_X->addGraph();
    ui->QCP_BODY_VEL_X->graph(0)->setPen(myPen);
    ui->QCP_BODY_VEL_X->graph(0)->setName("Current Vx");
    ui->QCP_BODY_VEL_X->addGraph();
    ui->QCP_BODY_VEL_X->graph(1)->setPen(dotPen);
    ui->QCP_BODY_VEL_X->graph(1)->setName("Desired Vx");
    ui->QCP_BODY_VEL_X->addGraph();
    ui->QCP_BODY_VEL_X->graph(2)->setPen(desiredPen);
    ui->QCP_BODY_VEL_X->graph(2)->setName("base line Vx");

    ui->QCP_BODY_VEL_Y->legend->setVisible(true);
    ui->QCP_BODY_VEL_Y->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_BODY_VEL_Y->addGraph();
    ui->QCP_BODY_VEL_Y->graph(0)->setPen(myPen);
    ui->QCP_BODY_VEL_Y->graph(0)->setName("Current Vy");
    ui->QCP_BODY_VEL_Y->addGraph();
    ui->QCP_BODY_VEL_Y->graph(1)->setPen(dotPen);
    ui->QCP_BODY_VEL_Y->graph(1)->setName("Desired Vy");
    ui->QCP_BODY_VEL_Y->addGraph();
    ui->QCP_BODY_VEL_Y->graph(2)->setPen(desiredPen);
    ui->QCP_BODY_VEL_Y->graph(2)->setName("base line Vy");

    ui->QCP_BODY_VEL_Z->legend->setVisible(true);
    ui->QCP_BODY_VEL_Z->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_BODY_VEL_Z->addGraph();
    ui->QCP_BODY_VEL_Z->graph(0)->setPen(myPen);
    ui->QCP_BODY_VEL_Z->graph(0)->setName("Current Vz");
    ui->QCP_BODY_VEL_Z->addGraph();
    ui->QCP_BODY_VEL_Z->graph(1)->setPen(dotPen);
    ui->QCP_BODY_VEL_Z->graph(1)->setName("Desired Vz");
    ui->QCP_BODY_VEL_Z->addGraph();
    ui->QCP_BODY_VEL_Z->graph(2)->setPen(desiredPen);
    ui->QCP_BODY_VEL_Z->graph(2)->setName("base line Vz");

    ui->QCP_BODY_ANG_POS_X->legend->setVisible(true);
    ui->QCP_BODY_ANG_POS_X->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_BODY_ANG_POS_X->addGraph();
    ui->QCP_BODY_ANG_POS_X->graph(0)->setPen(myPen);
    ui->QCP_BODY_ANG_POS_X->graph(0)->setName("Current roll");
    ui->QCP_BODY_ANG_POS_X->addGraph();
    ui->QCP_BODY_ANG_POS_X->graph(1)->setPen(desiredPen);
    ui->QCP_BODY_ANG_POS_X->graph(1)->setName("Desired roll");

    ui->QCP_BODY_ANG_POS_Y->legend->setVisible(true);
    ui->QCP_BODY_ANG_POS_Y->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_BODY_ANG_POS_Y->addGraph();
    ui->QCP_BODY_ANG_POS_Y->graph(0)->setPen(myPen);
    ui->QCP_BODY_ANG_POS_Y->graph(0)->setName("Current pitch");
    ui->QCP_BODY_ANG_POS_Y->addGraph();
    ui->QCP_BODY_ANG_POS_Y->graph(1)->setPen(desiredPen);
    ui->QCP_BODY_ANG_POS_Y->graph(01)->setName("Desired pitch");

    ui->QCP_BODY_ANG_POS_Z->legend->setVisible(true);
    ui->QCP_BODY_ANG_POS_Z->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_BODY_ANG_POS_Z->addGraph();
    ui->QCP_BODY_ANG_POS_Z->graph(0)->setPen(myPen);
    ui->QCP_BODY_ANG_POS_Z->graph(0)->setName("Current yaw");
    ui->QCP_BODY_ANG_POS_Z->addGraph();
    ui->QCP_BODY_ANG_POS_Z->graph(1)->setPen(desiredPen);
    ui->QCP_BODY_ANG_POS_Z->graph(1)->setName("Desired yaw");

    ui->QCP_BODY_ANG_VEL_X->legend->setVisible(true);
    ui->QCP_BODY_ANG_VEL_X->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_BODY_ANG_VEL_X->addGraph();
    ui->QCP_BODY_ANG_VEL_X->graph(0)->setPen(myPen);
    ui->QCP_BODY_ANG_VEL_X->graph(0)->setName("Current Wx");
    ui->QCP_BODY_ANG_VEL_X->addGraph();
    ui->QCP_BODY_ANG_VEL_X->graph(1)->setPen(desiredPen);
    ui->QCP_BODY_ANG_VEL_X->graph(1)->setName("Desired Wx");

    ui->QCP_BODY_ANG_VEL_Y->legend->setVisible(true);
    ui->QCP_BODY_ANG_VEL_Y->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_BODY_ANG_VEL_Y->addGraph();
    ui->QCP_BODY_ANG_VEL_Y->graph(0)->setPen(myPen);
    ui->QCP_BODY_ANG_VEL_Y->graph(0)->setName("Current Wy");
    ui->QCP_BODY_ANG_VEL_Y->addGraph();
    ui->QCP_BODY_ANG_VEL_Y->graph(1)->setPen(desiredPen);
    ui->QCP_BODY_ANG_VEL_Y->graph(1)->setName("Desired Wy");

    ui->QCP_BODY_ANG_VEL_Z->legend->setVisible(true);
    ui->QCP_BODY_ANG_VEL_Z->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_BODY_ANG_VEL_Z->addGraph();
    ui->QCP_BODY_ANG_VEL_Z->graph(0)->setPen(myPen);
    ui->QCP_BODY_ANG_VEL_Z->graph(0)->setName("Current Wz");
    ui->QCP_BODY_ANG_VEL_Z->addGraph();
    ui->QCP_BODY_ANG_VEL_Z->graph(1)->setPen(desiredPen);
    ui->QCP_BODY_ANG_VEL_Z->graph(1)->setName("Desired Wz");

    myPen.setColor(Qt::darkBlue);
    ui->QCP_JOYSTICK_LEFT->addGraph();
    ui->QCP_JOYSTICK_LEFT->graph(0)->setPen(myPen);
    ui->QCP_JOYSTICK_LEFT->graph(0)->setLineStyle((QCPGraph::LineStyle)0);
    ui->QCP_JOYSTICK_LEFT->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 40));

    ui->QCP_JOYSTICK_RIGHT->addGraph();
    ui->QCP_JOYSTICK_RIGHT->graph(0)->setPen(myPen);
    ui->QCP_JOYSTICK_RIGHT->graph(0)->setLineStyle((QCPGraph::LineStyle)0);
    ui->QCP_JOYSTICK_RIGHT->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 40));


    ui->QCP_BODY_POS_X->xAxis->setTicker(timeTicker);
    ui->QCP_BODY_POS_Y->xAxis->setTicker(timeTicker);
    ui->QCP_BODY_POS_Z->xAxis->setTicker(timeTicker);
    ui->QCP_BODY_VEL_X->xAxis->setTicker(timeTicker);
    ui->QCP_BODY_VEL_Y->xAxis->setTicker(timeTicker);
    ui->QCP_BODY_VEL_Z->xAxis->setTicker(timeTicker);

    ui->QCP_BODY_ANG_POS_X->xAxis->setTicker(timeTicker);
    ui->QCP_BODY_ANG_POS_Y->xAxis->setTicker(timeTicker);
    ui->QCP_BODY_ANG_POS_Z->xAxis->setTicker(timeTicker);
    ui->QCP_BODY_ANG_VEL_X->xAxis->setTicker(timeTicker);
    ui->QCP_BODY_ANG_VEL_Y->xAxis->setTicker(timeTicker);
    ui->QCP_BODY_ANG_VEL_Z->xAxis->setTicker(timeTicker);


    ui->QCP_BODY_VEL_X->yAxis->setRange(-3.0, 3.0);
    ui->QCP_BODY_VEL_Y->yAxis->setRange(-3.0, 3.0);
    ui->QCP_BODY_VEL_Z->yAxis->setRange(-1.0, 1.0);

    ui->QCP_BODY_ANG_POS_X->yAxis->setRange(-1.0, 1.0);
    ui->QCP_BODY_ANG_POS_Y->yAxis->setRange(-1.0, 1.0);
    ui->QCP_BODY_ANG_VEL_X->yAxis->setRange(-2.0, 2.0);
    ui->QCP_BODY_ANG_VEL_Y->yAxis->setRange(-2.0, 2.0);
    ui->QCP_BODY_ANG_VEL_Z->yAxis->setRange(-2.0, 2.0);


    ui->QCP_JOYSTICK_LEFT->xAxis->setRange(-1.0, 1.0);
    ui->QCP_JOYSTICK_LEFT->yAxis->setRange(-1.0, 1.0);
    ui->QCP_JOYSTICK_RIGHT->xAxis->setRange(-1.0, 1.0);
    ui->QCP_JOYSTICK_RIGHT->yAxis->setRange(-1.0, 1.0);

    ui->QCP_JOYSTICK_LEFT->xAxis->setVisible(false);
    ui->QCP_JOYSTICK_LEFT->yAxis->setVisible(false);
    ui->QCP_JOYSTICK_RIGHT->xAxis->setVisible(false);
    ui->QCP_JOYSTICK_RIGHT->yAxis->setVisible(false);

    ui->QCP_MT_POS_FLHR->legend->setVisible(true);
    ui->QCP_MT_POS_FLHR->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_POS_FLHR->addGraph();
    ui->QCP_MT_POS_FLHR->graph(0)->setPen(myPen);
    ui->QCP_MT_POS_FLHR->graph(0)->setName("Current");
    ui->QCP_MT_POS_FLHR->addGraph();
    ui->QCP_MT_POS_FLHR->graph(1)->setPen(desiredPen);
    ui->QCP_MT_POS_FLHR->graph(1)->setName("Desired");

    ui->QCP_MT_POS_FLHP->legend->setVisible(true);
    ui->QCP_MT_POS_FLHP->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_POS_FLHP->addGraph();
    ui->QCP_MT_POS_FLHP->graph(0)->setPen(myPen);
    ui->QCP_MT_POS_FLHP->graph(0)->setName("Current");
    ui->QCP_MT_POS_FLHP->addGraph();
    ui->QCP_MT_POS_FLHP->graph(1)->setPen(desiredPen);
    ui->QCP_MT_POS_FLHP->graph(1)->setName("Desired");

    ui->QCP_MT_POS_FLKP->legend->setVisible(true);
    ui->QCP_MT_POS_FLKP->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_POS_FLKP->addGraph();
    ui->QCP_MT_POS_FLKP->graph(0)->setPen(myPen);
    ui->QCP_MT_POS_FLKP->graph(0)->setName("Current");
    ui->QCP_MT_POS_FLKP->addGraph();
    ui->QCP_MT_POS_FLKP->graph(1)->setPen(desiredPen);
    ui->QCP_MT_POS_FLKP->graph(1)->setName("Desired");

    ui->QCP_MT_POS_FRHR->legend->setVisible(true);
    ui->QCP_MT_POS_FRHR->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_POS_FRHR->addGraph();
    ui->QCP_MT_POS_FRHR->graph(0)->setPen(myPen);
    ui->QCP_MT_POS_FRHR->graph(0)->setName("Current");
    ui->QCP_MT_POS_FRHR->addGraph();
    ui->QCP_MT_POS_FRHR->graph(1)->setPen(desiredPen);
    ui->QCP_MT_POS_FRHR->graph(1)->setName("Desired");

    ui->QCP_MT_POS_FRHP->legend->setVisible(true);
    ui->QCP_MT_POS_FRHP->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_POS_FRHP->addGraph();
    ui->QCP_MT_POS_FRHP->graph(0)->setPen(myPen);
    ui->QCP_MT_POS_FRHP->graph(0)->setName("Current");
    ui->QCP_MT_POS_FRHP->addGraph();
    ui->QCP_MT_POS_FRHP->graph(1)->setPen(desiredPen);
    ui->QCP_MT_POS_FRHP->graph(1)->setName("Desired");

    ui->QCP_MT_POS_FRKP->legend->setVisible(true);
    ui->QCP_MT_POS_FRKP->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_POS_FRKP->addGraph();
    ui->QCP_MT_POS_FRKP->graph(0)->setPen(myPen);
    ui->QCP_MT_POS_FRKP->graph(0)->setName("Current");
    ui->QCP_MT_POS_FRKP->addGraph();
    ui->QCP_MT_POS_FRKP->graph(1)->setPen(desiredPen);
    ui->QCP_MT_POS_FRKP->graph(1)->setName("Desired");

    ui->QCP_MT_POS_HLHR->legend->setVisible(true);
    ui->QCP_MT_POS_HLHR->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_POS_HLHR->addGraph();
    ui->QCP_MT_POS_HLHR->graph(0)->setPen(myPen);
    ui->QCP_MT_POS_HLHR->graph(0)->setName("Current");
    ui->QCP_MT_POS_HLHR->addGraph();
    ui->QCP_MT_POS_HLHR->graph(1)->setPen(desiredPen);
    ui->QCP_MT_POS_HLHR->graph(1)->setName("Desired");

    ui->QCP_MT_POS_HLHP->legend->setVisible(true);
    ui->QCP_MT_POS_HLHP->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_POS_HLHP->addGraph();
    ui->QCP_MT_POS_HLHP->graph(0)->setPen(myPen);
    ui->QCP_MT_POS_HLHP->graph(0)->setName("Current");
    ui->QCP_MT_POS_HLHP->addGraph();
    ui->QCP_MT_POS_HLHP->graph(1)->setPen(desiredPen);
    ui->QCP_MT_POS_HLHP->graph(1)->setName("Desired");

    ui->QCP_MT_POS_HLKP->legend->setVisible(true);
    ui->QCP_MT_POS_HLKP->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_POS_HLKP->addGraph();
    ui->QCP_MT_POS_HLKP->graph(0)->setPen(myPen);
    ui->QCP_MT_POS_HLKP->graph(0)->setName("Current");
    ui->QCP_MT_POS_HLKP->addGraph();
    ui->QCP_MT_POS_HLKP->graph(1)->setPen(desiredPen);
    ui->QCP_MT_POS_HLKP->graph(1)->setName("Desired");

    ui->QCP_MT_POS_HRHR->legend->setVisible(true);
    ui->QCP_MT_POS_HRHR->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_POS_HRHR->addGraph();
    ui->QCP_MT_POS_HRHR->graph(0)->setPen(myPen);
    ui->QCP_MT_POS_HRHR->graph(0)->setName("Current");
    ui->QCP_MT_POS_HRHR->addGraph();
    ui->QCP_MT_POS_HRHR->graph(1)->setPen(desiredPen);
    ui->QCP_MT_POS_HRHR->graph(1)->setName("Desired");

    ui->QCP_MT_POS_HRHP->legend->setVisible(true);
    ui->QCP_MT_POS_HRHP->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_POS_HRHP->addGraph();
    ui->QCP_MT_POS_HRHP->graph(0)->setPen(myPen);
    ui->QCP_MT_POS_HRHP->graph(0)->setName("Current");
    ui->QCP_MT_POS_HRHP->addGraph();
    ui->QCP_MT_POS_HRHP->graph(1)->setPen(desiredPen);
    ui->QCP_MT_POS_HRHP->graph(1)->setName("Desired");

    ui->QCP_MT_POS_HRKP->legend->setVisible(true);
    ui->QCP_MT_POS_HRKP->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_POS_HRKP->addGraph();
    ui->QCP_MT_POS_HRKP->graph(0)->setPen(myPen);
    ui->QCP_MT_POS_HRKP->graph(0)->setName("Current");
    ui->QCP_MT_POS_HRKP->addGraph();
    ui->QCP_MT_POS_HRKP->graph(1)->setPen(desiredPen);
    ui->QCP_MT_POS_HRKP->graph(1)->setName("Desired");

    ui->QCP_MT_VEL_FLHR->legend->setVisible(true);
    ui->QCP_MT_VEL_FLHR->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_VEL_FLHR->addGraph();
    ui->QCP_MT_VEL_FLHR->graph(0)->setPen(myPen);
    ui->QCP_MT_VEL_FLHR->graph(0)->setName("Current");
    ui->QCP_MT_VEL_FLHR->addGraph();
    ui->QCP_MT_VEL_FLHR->graph(1)->setPen(desiredPen);
    ui->QCP_MT_VEL_FLHR->graph(1)->setName("Desired");

    ui->QCP_MT_VEL_FLHP->legend->setVisible(true);
    ui->QCP_MT_VEL_FLHP->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_VEL_FLHP->addGraph();
    ui->QCP_MT_VEL_FLHP->graph(0)->setPen(myPen);
    ui->QCP_MT_VEL_FLHP->graph(0)->setName("Current");
    ui->QCP_MT_VEL_FLHP->addGraph();
    ui->QCP_MT_VEL_FLHP->graph(1)->setPen(desiredPen);
    ui->QCP_MT_VEL_FLHP->graph(1)->setName("Desired");

    ui->QCP_MT_VEL_FLKP->legend->setVisible(true);
    ui->QCP_MT_VEL_FLKP->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_VEL_FLKP->addGraph();
    ui->QCP_MT_VEL_FLKP->graph(0)->setPen(myPen);
    ui->QCP_MT_VEL_FLKP->graph(0)->setName("Current");
    ui->QCP_MT_VEL_FLKP->addGraph();
    ui->QCP_MT_VEL_FLKP->graph(1)->setPen(desiredPen);
    ui->QCP_MT_VEL_FLKP->graph(1)->setName("Desired");

    ui->QCP_MT_VEL_FRHR->legend->setVisible(true);
    ui->QCP_MT_VEL_FRHR->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_VEL_FRHR->addGraph();
    ui->QCP_MT_VEL_FRHR->graph(0)->setPen(myPen);
    ui->QCP_MT_VEL_FRHR->graph(0)->setName("Current");
    ui->QCP_MT_VEL_FRHR->addGraph();
    ui->QCP_MT_VEL_FRHR->graph(1)->setPen(desiredPen);
    ui->QCP_MT_VEL_FRHR->graph(1)->setName("Desired");

    ui->QCP_MT_VEL_FRHP->legend->setVisible(true);
    ui->QCP_MT_VEL_FRHP->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_VEL_FRHP->addGraph();
    ui->QCP_MT_VEL_FRHP->graph(0)->setPen(myPen);
    ui->QCP_MT_VEL_FRHP->graph(0)->setName("Current");
    ui->QCP_MT_VEL_FRHP->addGraph();
    ui->QCP_MT_VEL_FRHP->graph(1)->setPen(desiredPen);
    ui->QCP_MT_VEL_FRHP->graph(1)->setName("Desired");

    ui->QCP_MT_VEL_FRKP->legend->setVisible(true);
    ui->QCP_MT_VEL_FRKP->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_VEL_FRKP->addGraph();
    ui->QCP_MT_VEL_FRKP->graph(0)->setPen(myPen);
    ui->QCP_MT_VEL_FRKP->graph(0)->setName("Current");
    ui->QCP_MT_VEL_FRKP->addGraph();
    ui->QCP_MT_VEL_FRKP->graph(1)->setPen(desiredPen);
    ui->QCP_MT_VEL_FRKP->graph(1)->setName("Desired");

    ui->QCP_MT_VEL_HLHR->legend->setVisible(true);
    ui->QCP_MT_VEL_HLHR->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_VEL_HLHR->addGraph();
    ui->QCP_MT_VEL_HLHR->graph(0)->setPen(myPen);
    ui->QCP_MT_VEL_HLHR->graph(0)->setName("Current");
    ui->QCP_MT_VEL_HLHR->addGraph();
    ui->QCP_MT_VEL_HLHR->graph(1)->setPen(desiredPen);
    ui->QCP_MT_VEL_HLHR->graph(1)->setName("Desired");

    ui->QCP_MT_VEL_HLHP->legend->setVisible(true);
    ui->QCP_MT_VEL_HLHP->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_VEL_HLHP->addGraph();
    ui->QCP_MT_VEL_HLHP->graph(0)->setPen(myPen);
    ui->QCP_MT_VEL_HLHP->graph(0)->setName("Current");
    ui->QCP_MT_VEL_HLHP->addGraph();
    ui->QCP_MT_VEL_HLHP->graph(1)->setPen(desiredPen);
    ui->QCP_MT_VEL_HLHP->graph(1)->setName("Desired");

    ui->QCP_MT_VEL_HLKP->legend->setVisible(true);
    ui->QCP_MT_VEL_HLKP->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_VEL_HLKP->addGraph();
    ui->QCP_MT_VEL_HLKP->graph(0)->setPen(myPen);
    ui->QCP_MT_VEL_HLKP->graph(0)->setName("Current");
    ui->QCP_MT_VEL_HLKP->addGraph();
    ui->QCP_MT_VEL_HLKP->graph(1)->setPen(desiredPen);
    ui->QCP_MT_VEL_HLKP->graph(1)->setName("Desired");

    ui->QCP_MT_VEL_HRHR->legend->setVisible(true);
    ui->QCP_MT_VEL_HRHR->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_VEL_HRHR->addGraph();
    ui->QCP_MT_VEL_HRHR->graph(0)->setPen(myPen);
    ui->QCP_MT_VEL_HRHR->graph(0)->setName("Current");
    ui->QCP_MT_VEL_HRHR->addGraph();
    ui->QCP_MT_VEL_HRHR->graph(1)->setPen(desiredPen);
    ui->QCP_MT_VEL_HRHR->graph(1)->setName("Desired");

    ui->QCP_MT_VEL_HRHP->legend->setVisible(true);
    ui->QCP_MT_VEL_HRHP->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_VEL_HRHP->addGraph();
    ui->QCP_MT_VEL_HRHP->graph(0)->setPen(myPen);
    ui->QCP_MT_VEL_HRHP->graph(0)->setName("Current");
    ui->QCP_MT_VEL_HRHP->addGraph();
    ui->QCP_MT_VEL_HRHP->graph(1)->setPen(desiredPen);
    ui->QCP_MT_VEL_HRHP->graph(1)->setName("Desired");

    ui->QCP_MT_VEL_HRKP->legend->setVisible(true);
    ui->QCP_MT_VEL_HRKP->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_VEL_HRKP->addGraph();
    ui->QCP_MT_VEL_HRKP->graph(0)->setPen(myPen);
    ui->QCP_MT_VEL_HRKP->graph(0)->setName("Current");
    ui->QCP_MT_VEL_HRKP->addGraph();
    ui->QCP_MT_VEL_HRKP->graph(1)->setPen(desiredPen);
    ui->QCP_MT_VEL_HRKP->graph(1)->setName("Desired");

    ui->QCP_MT_TAU_FLHR->legend->setVisible(true);
    ui->QCP_MT_TAU_FLHR->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_TAU_FLHR->addGraph();
    ui->QCP_MT_TAU_FLHR->graph(0)->setPen(myPen);
    ui->QCP_MT_TAU_FLHR->graph(0)->setName("Current");
    ui->QCP_MT_TAU_FLHR->addGraph();
    ui->QCP_MT_TAU_FLHR->graph(1)->setPen(desiredPen);
    ui->QCP_MT_TAU_FLHR->graph(1)->setName("Desired");

    ui->QCP_MT_TAU_FLHP->legend->setVisible(true);
    ui->QCP_MT_TAU_FLHP->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_TAU_FLHP->addGraph();
    ui->QCP_MT_TAU_FLHP->graph(0)->setPen(myPen);
    ui->QCP_MT_TAU_FLHP->graph(0)->setName("Current");
    ui->QCP_MT_TAU_FLHP->addGraph();
    ui->QCP_MT_TAU_FLHP->graph(1)->setPen(desiredPen);
    ui->QCP_MT_TAU_FLHP->graph(1)->setName("Desired");

    ui->QCP_MT_TAU_FLKP->legend->setVisible(true);
    ui->QCP_MT_TAU_FLKP->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_TAU_FLKP->addGraph();
    ui->QCP_MT_TAU_FLKP->graph(0)->setPen(myPen);
    ui->QCP_MT_TAU_FLKP->graph(0)->setName("Current");
    ui->QCP_MT_TAU_FLKP->addGraph();
    ui->QCP_MT_TAU_FLKP->graph(1)->setPen(desiredPen);
    ui->QCP_MT_TAU_FLKP->graph(1)->setName("Desired");

    ui->QCP_MT_TAU_FRHR->legend->setVisible(true);
    ui->QCP_MT_TAU_FRHR->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_TAU_FRHR->addGraph();
    ui->QCP_MT_TAU_FRHR->graph(0)->setPen(myPen);
    ui->QCP_MT_TAU_FRHR->graph(0)->setName("Current");
    ui->QCP_MT_TAU_FRHR->addGraph();
    ui->QCP_MT_TAU_FRHR->graph(1)->setPen(desiredPen);
    ui->QCP_MT_TAU_FRHR->graph(1)->setName("Desired");

    ui->QCP_MT_TAU_FRHP->legend->setVisible(true);
    ui->QCP_MT_TAU_FRHP->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_TAU_FRHP->addGraph();
    ui->QCP_MT_TAU_FRHP->graph(0)->setPen(myPen);
    ui->QCP_MT_TAU_FRHP->graph(0)->setName("Current");
    ui->QCP_MT_TAU_FRHP->addGraph();
    ui->QCP_MT_TAU_FRHP->graph(1)->setPen(desiredPen);
    ui->QCP_MT_TAU_FRHP->graph(1)->setName("Desired");

    ui->QCP_MT_TAU_FRKP->legend->setVisible(true);
    ui->QCP_MT_TAU_FRKP->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_TAU_FRKP->addGraph();
    ui->QCP_MT_TAU_FRKP->graph(0)->setPen(myPen);
    ui->QCP_MT_TAU_FRKP->graph(0)->setName("Current");
    ui->QCP_MT_TAU_FRKP->addGraph();
    ui->QCP_MT_TAU_FRKP->graph(1)->setPen(desiredPen);
    ui->QCP_MT_TAU_FRKP->graph(1)->setName("Desired");

    ui->QCP_MT_TAU_HLHR->legend->setVisible(true);
    ui->QCP_MT_TAU_HLHR->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_TAU_HLHR->addGraph();
    ui->QCP_MT_TAU_HLHR->graph(0)->setPen(myPen);
    ui->QCP_MT_TAU_HLHR->graph(0)->setName("Current");
    ui->QCP_MT_TAU_HLHR->addGraph();
    ui->QCP_MT_TAU_HLHR->graph(1)->setPen(desiredPen);
    ui->QCP_MT_TAU_HLHR->graph(1)->setName("Desired");

    ui->QCP_MT_TAU_HLHP->legend->setVisible(true);
    ui->QCP_MT_TAU_HLHP->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_TAU_HLHP->addGraph();
    ui->QCP_MT_TAU_HLHP->graph(0)->setPen(myPen);
    ui->QCP_MT_TAU_HLHP->graph(0)->setName("Current");
    ui->QCP_MT_TAU_HLHP->addGraph();
    ui->QCP_MT_TAU_HLHP->graph(1)->setPen(desiredPen);
    ui->QCP_MT_TAU_HLHP->graph(1)->setName("Desired");

    ui->QCP_MT_TAU_HLKP->legend->setVisible(true);
    ui->QCP_MT_TAU_HLKP->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_TAU_HLKP->addGraph();
    ui->QCP_MT_TAU_HLKP->graph(0)->setPen(myPen);
    ui->QCP_MT_TAU_HLKP->graph(0)->setName("Current");
    ui->QCP_MT_TAU_HLKP->addGraph();
    ui->QCP_MT_TAU_HLKP->graph(1)->setPen(desiredPen);
    ui->QCP_MT_TAU_HLKP->graph(1)->setName("Desired");

    ui->QCP_MT_TAU_HRHR->legend->setVisible(true);
    ui->QCP_MT_TAU_HRHR->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_TAU_HRHR->addGraph();
    ui->QCP_MT_TAU_HRHR->graph(0)->setPen(myPen);
    ui->QCP_MT_TAU_HRHR->graph(0)->setName("Current");
    ui->QCP_MT_TAU_HRHR->addGraph();
    ui->QCP_MT_TAU_HRHR->graph(1)->setPen(desiredPen);
    ui->QCP_MT_TAU_HRHR->graph(1)->setName("Desired");

    ui->QCP_MT_TAU_HRHP->legend->setVisible(true);
    ui->QCP_MT_TAU_HRHP->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_TAU_HRHP->addGraph();
    ui->QCP_MT_TAU_HRHP->graph(0)->setPen(myPen);
    ui->QCP_MT_TAU_HRHP->graph(0)->setName("Current");
    ui->QCP_MT_TAU_HRHP->addGraph();
    ui->QCP_MT_TAU_HRHP->graph(1)->setPen(desiredPen);
    ui->QCP_MT_TAU_HRHP->graph(1)->setName("Desired");

    ui->QCP_MT_TAU_HRKP->legend->setVisible(true);
    ui->QCP_MT_TAU_HRKP->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_MT_TAU_HRKP->addGraph();
    ui->QCP_MT_TAU_HRKP->graph(0)->setPen(myPen);
    ui->QCP_MT_TAU_HRKP->graph(0)->setName("Current");
    ui->QCP_MT_TAU_HRKP->addGraph();
    ui->QCP_MT_TAU_HRKP->graph(1)->setPen(desiredPen);
    ui->QCP_MT_TAU_HRKP->graph(1)->setName("Desired");
    ui->QCP_CONTACT_FL->legend->setVisible(true);
    ui->QCP_CONTACT_FL->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_CONTACT_FR->legend->setVisible(true);
    ui->QCP_CONTACT_FR->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_CONTACT_HL->legend->setVisible(true);
    ui->QCP_CONTACT_HL->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_CONTACT_HR->legend->setVisible(true);
    ui->QCP_CONTACT_HR->legend->setFont(QFont("Helvetica", fontsize));
    ui->QCP_CONTACT_FL->addGraph();
    ui->QCP_CONTACT_FL->graph(0)->setPen(myPen);
    ui->QCP_CONTACT_FL->graph(0)->setName("Contact");
    ui->QCP_CONTACT_FR->addGraph();
    ui->QCP_CONTACT_FR->graph(0)->setPen(myPen);
    ui->QCP_CONTACT_FR->graph(0)->setName("Contact");
    ui->QCP_CONTACT_HL->addGraph();
    ui->QCP_CONTACT_HL->graph(0)->setPen(myPen);
    ui->QCP_CONTACT_HL->graph(0)->setName("Contact");
    ui->QCP_CONTACT_HR->addGraph();
    ui->QCP_CONTACT_HR->graph(0)->setPen(myPen);
    ui->QCP_CONTACT_HR->graph(0)->setName("Contact");
    myPen.setColor(Qt::red);
    ui->QCP_CONTACT_FL->addGraph();
    ui->QCP_CONTACT_FL->graph(1)->setPen(myPen);
    ui->QCP_CONTACT_FL->graph(1)->setName("Gait");
    ui->QCP_CONTACT_FR->addGraph();
    ui->QCP_CONTACT_FR->graph(1)->setPen(myPen);
    ui->QCP_CONTACT_FR->graph(1)->setName("Gait");
    ui->QCP_CONTACT_HL->addGraph();
    ui->QCP_CONTACT_HL->graph(1)->setPen(myPen);
    ui->QCP_CONTACT_HL->graph(1)->setName("Gait");
    ui->QCP_CONTACT_HR->addGraph();
    ui->QCP_CONTACT_HR->graph(1)->setPen(myPen);
    ui->QCP_CONTACT_HR->graph(1)->setName("Gait");

    myPen.setColor(Qt::darkGreen);
    ui->QCP_CONTACT_FL->addGraph();
    ui->QCP_CONTACT_FL->graph(2)->setPen(myPen);
    ui->QCP_CONTACT_FL->graph(2)->setName("Residual");
    ui->QCP_CONTACT_FR->addGraph();
    ui->QCP_CONTACT_FR->graph(2)->setPen(myPen);
    ui->QCP_CONTACT_FR->graph(2)->setName("Residual");
    ui->QCP_CONTACT_HL->addGraph();
    ui->QCP_CONTACT_HL->graph(2)->setPen(myPen);
    ui->QCP_CONTACT_HL->graph(2)->setName("Residual");
    ui->QCP_CONTACT_HR->addGraph();
    ui->QCP_CONTACT_HR->graph(2)->setPen(myPen);
    ui->QCP_CONTACT_HR->graph(2)->setName("Residual");

    ui->QCP_MT_POS_FLHR->xAxis->setTicker(timeTicker);
    ui->QCP_MT_POS_FLHP->xAxis->setTicker(timeTicker);
    ui->QCP_MT_POS_FLKP->xAxis->setTicker(timeTicker);
    ui->QCP_MT_POS_FRHR->xAxis->setTicker(timeTicker);
    ui->QCP_MT_POS_FRHP->xAxis->setTicker(timeTicker);
    ui->QCP_MT_POS_FRKP->xAxis->setTicker(timeTicker);
    ui->QCP_MT_POS_HLHR->xAxis->setTicker(timeTicker);
    ui->QCP_MT_POS_HLHP->xAxis->setTicker(timeTicker);
    ui->QCP_MT_POS_HLKP->xAxis->setTicker(timeTicker);
    ui->QCP_MT_POS_HRHR->xAxis->setTicker(timeTicker);
    ui->QCP_MT_POS_HRHP->xAxis->setTicker(timeTicker);
    ui->QCP_MT_POS_HRKP->xAxis->setTicker(timeTicker);

    ui->QCP_MT_VEL_FLHR->xAxis->setTicker(timeTicker);
    ui->QCP_MT_VEL_FLHP->xAxis->setTicker(timeTicker);
    ui->QCP_MT_VEL_FLKP->xAxis->setTicker(timeTicker);
    ui->QCP_MT_VEL_FRHR->xAxis->setTicker(timeTicker);
    ui->QCP_MT_VEL_FRHP->xAxis->setTicker(timeTicker);
    ui->QCP_MT_VEL_FRKP->xAxis->setTicker(timeTicker);
    ui->QCP_MT_VEL_HLHR->xAxis->setTicker(timeTicker);
    ui->QCP_MT_VEL_HLHP->xAxis->setTicker(timeTicker);
    ui->QCP_MT_VEL_HLKP->xAxis->setTicker(timeTicker);
    ui->QCP_MT_VEL_HRHR->xAxis->setTicker(timeTicker);
    ui->QCP_MT_VEL_HRHP->xAxis->setTicker(timeTicker);
    ui->QCP_MT_VEL_HRKP->xAxis->setTicker(timeTicker);

    ui->QCP_MT_TAU_FLHR->xAxis->setTicker(timeTicker);
    ui->QCP_MT_TAU_FLHP->xAxis->setTicker(timeTicker);
    ui->QCP_MT_TAU_FLKP->xAxis->setTicker(timeTicker);
    ui->QCP_MT_TAU_FRHR->xAxis->setTicker(timeTicker);
    ui->QCP_MT_TAU_FRHP->xAxis->setTicker(timeTicker);
    ui->QCP_MT_TAU_FRKP->xAxis->setTicker(timeTicker);
    ui->QCP_MT_TAU_HLHR->xAxis->setTicker(timeTicker);
    ui->QCP_MT_TAU_HLHP->xAxis->setTicker(timeTicker);
    ui->QCP_MT_TAU_HLKP->xAxis->setTicker(timeTicker);
    ui->QCP_MT_TAU_HRHR->xAxis->setTicker(timeTicker);
    ui->QCP_MT_TAU_HRHP->xAxis->setTicker(timeTicker);
    ui->QCP_MT_TAU_HRKP->xAxis->setTicker(timeTicker);

    ui->QCP_MT_POS_FLHR->yAxis->setRange(-0.3, 0.3);
    ui->QCP_MT_POS_FRHR->yAxis->setRange(-0.3, 0.3);
    ui->QCP_MT_POS_HLHR->yAxis->setRange(-0.3, 0.3);
    ui->QCP_MT_POS_HRHR->yAxis->setRange(-0.3, 0.3);
    ui->QCP_MT_POS_FLHP->yAxis->setRange(0.0, 3.0);
    ui->QCP_MT_POS_FRHP->yAxis->setRange(0.0, 3.0);
    ui->QCP_MT_POS_HLHP->yAxis->setRange(0.0, 3.0);
    ui->QCP_MT_POS_HRHP->yAxis->setRange(0.0, 3.0);
    ui->QCP_MT_POS_FLKP->yAxis->setRange(-3.0, 0.0);
    ui->QCP_MT_POS_FRKP->yAxis->setRange(-3.0, 0.0);
    ui->QCP_MT_POS_HLKP->yAxis->setRange(-3.0, 0.0);
    ui->QCP_MT_POS_HRKP->yAxis->setRange(-3.0, 0.0);

    ui->QCP_MT_VEL_FLHR->yAxis->setRange(-5.0, 5.0);
    ui->QCP_MT_VEL_FRHR->yAxis->setRange(-5.0, 5.0);
    ui->QCP_MT_VEL_HLHR->yAxis->setRange(-5.0, 5.0);
    ui->QCP_MT_VEL_HRHR->yAxis->setRange(-5.0, 5.0);
    ui->QCP_MT_VEL_FLHP->yAxis->setRange(-15.0, 15.0);
    ui->QCP_MT_VEL_FRHP->yAxis->setRange(-15.0, 15.0);
    ui->QCP_MT_VEL_HLHP->yAxis->setRange(-15.0, 15.0);
    ui->QCP_MT_VEL_HRHP->yAxis->setRange(-15.0, 15.0);
    ui->QCP_MT_VEL_FLKP->yAxis->setRange(-15.0, 15.0);
    ui->QCP_MT_VEL_FRKP->yAxis->setRange(-15.0, 15.0);
    ui->QCP_MT_VEL_HLKP->yAxis->setRange(-15.0, 15.0);
    ui->QCP_MT_VEL_HRKP->yAxis->setRange(-15.0, 15.0);

    ui->QCP_MT_TAU_FLHR->yAxis->setRange(-10.0, 10.0);
    ui->QCP_MT_TAU_FRHR->yAxis->setRange(-10.0, 10.0);
    ui->QCP_MT_TAU_HLHR->yAxis->setRange(-10.0, 10.0);
    ui->QCP_MT_TAU_HRHR->yAxis->setRange(-10.0, 10.0);
    ui->QCP_MT_TAU_FLHP->yAxis->setRange(-30.0, 30.0);
    ui->QCP_MT_TAU_FRHP->yAxis->setRange(-30.0, 30.0);
    ui->QCP_MT_TAU_HLHP->yAxis->setRange(-30.0, 30.0);
    ui->QCP_MT_TAU_HRHP->yAxis->setRange(-30.0, 30.0);
    ui->QCP_MT_TAU_FLKP->yAxis->setRange(-30.0, 30.0);
    ui->QCP_MT_TAU_FRKP->yAxis->setRange(-30.0, 30.0);
    ui->QCP_MT_TAU_HLKP->yAxis->setRange(-30.0, 30.0);
    ui->QCP_MT_TAU_HRKP->yAxis->setRange(-30.0, 30.0);

    ui->QCP_CONTACT_FL->yAxis->setRange(-5, 25);
    ui->QCP_CONTACT_FR->yAxis->setRange(-5, 25);
    ui->QCP_CONTACT_HL->yAxis->setRange(-5, 25);
    ui->QCP_CONTACT_HR->yAxis->setRange(-5, 25);
}

void MainWindow::addGraphData()
{
    ui->QCP_BODY_POS_X->graph(0)->addData(sharedMemory->localTime, sharedMemory->globalBasePosition[0]);
    ui->QCP_BODY_POS_Y->graph(0)->addData(sharedMemory->localTime, sharedMemory->globalBasePosition[1]);
    ui->QCP_BODY_POS_Z->graph(0)->addData(sharedMemory->localTime, sharedMemory->globalBasePosition[2]);
    ui->QCP_BODY_VEL_X->graph(0)->addData(sharedMemory->localTime, sharedMemory->globalBaseVelocity[0]);
    ui->QCP_BODY_VEL_Y->graph(0)->addData(sharedMemory->localTime, sharedMemory->globalBaseVelocity[1]);
    ui->QCP_BODY_VEL_Z->graph(0)->addData(sharedMemory->localTime, sharedMemory->globalBaseVelocity[2]);

    ui->QCP_BODY_POS_X->graph(1)->addData(sharedMemory->localTime, sharedMemory->globalBaseDesiredPosition[0]);
    ui->QCP_BODY_POS_Y->graph(1)->addData(sharedMemory->localTime, sharedMemory->globalBaseDesiredPosition[1]);
    ui->QCP_BODY_POS_Z->graph(1)->addData(sharedMemory->localTime, sharedMemory->globalBaseDesiredPosition[2]);
    ui->QCP_BODY_VEL_X->graph(1)->addData(sharedMemory->localTime, sharedMemory->globalBaseDesiredVelocity[0]);
    ui->QCP_BODY_VEL_Y->graph(1)->addData(sharedMemory->localTime, sharedMemory->globalBaseDesiredVelocity[1]);
    ui->QCP_BODY_VEL_Z->graph(1)->addData(sharedMemory->localTime, sharedMemory->globalBaseDesiredVelocity[2]);

    ui->QCP_BODY_ANG_POS_X->graph(0)->addData(sharedMemory->localTime, sharedMemory->globalBaseEulerAngle[0]);
    ui->QCP_BODY_ANG_POS_Y->graph(0)->addData(sharedMemory->localTime, sharedMemory->globalBaseEulerAngle[1]);
    ui->QCP_BODY_ANG_POS_Z->graph(0)->addData(sharedMemory->localTime, sharedMemory->globalBaseEulerAngle[2]);
    ui->QCP_BODY_ANG_POS_X->graph(1)->addData(sharedMemory->localTime, sharedMemory->globalBaseDesiredEulerAngle[0]);
    ui->QCP_BODY_ANG_POS_Y->graph(1)->addData(sharedMemory->localTime, sharedMemory->globalBaseDesiredEulerAngle[1]);
    ui->QCP_BODY_ANG_POS_Z->graph(1)->addData(sharedMemory->localTime, sharedMemory->globalBaseDesiredEulerAngle[2]);

    ui->QCP_BODY_ANG_VEL_X->graph(0)->addData(sharedMemory->localTime, sharedMemory->bodyBaseAngularVelocity[0]);
    ui->QCP_BODY_ANG_VEL_Y->graph(0)->addData(sharedMemory->localTime, sharedMemory->bodyBaseAngularVelocity[1]);
    ui->QCP_BODY_ANG_VEL_Z->graph(0)->addData(sharedMemory->localTime, sharedMemory->bodyBaseAngularVelocity[2]);
    ui->QCP_BODY_ANG_VEL_X->graph(1)->addData(sharedMemory->localTime, sharedMemory->bodyBaseDesiredAngularVelocity[0]);
    ui->QCP_BODY_ANG_VEL_Y->graph(1)->addData(sharedMemory->localTime, sharedMemory->bodyBaseDesiredAngularVelocity[1]);
    ui->QCP_BODY_ANG_VEL_Z->graph(1)->addData(sharedMemory->localTime, sharedMemory->bodyBaseDesiredAngularVelocity[2]);

    ui->QCP_MT_POS_FLHR->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorPosition[MOTOR_INDEX::FLHR_IDX]);
    ui->QCP_MT_POS_FLHP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorPosition[MOTOR_INDEX::FLHP_IDX]);
    ui->QCP_MT_POS_FLKP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorPosition[MOTOR_INDEX::FLKP_IDX]);
    ui->QCP_MT_POS_FRHR->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorPosition[MOTOR_INDEX::FRHR_IDX]);
    ui->QCP_MT_POS_FRHP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorPosition[MOTOR_INDEX::FRHP_IDX]);
    ui->QCP_MT_POS_FRKP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorPosition[MOTOR_INDEX::FRKP_IDX]);
    ui->QCP_MT_POS_HLHR->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorPosition[MOTOR_INDEX::HLHR_IDX]);
    ui->QCP_MT_POS_HLHP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorPosition[MOTOR_INDEX::HLHP_IDX]);
    ui->QCP_MT_POS_HLKP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorPosition[MOTOR_INDEX::HLKP_IDX]);
    ui->QCP_MT_POS_HRHR->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorPosition[MOTOR_INDEX::HRHR_IDX]);
    ui->QCP_MT_POS_HRHP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorPosition[MOTOR_INDEX::HRHP_IDX]);
    ui->QCP_MT_POS_HRKP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorPosition[MOTOR_INDEX::HRKP_IDX]);
    ui->QCP_MT_POS_FLHR->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredPosition[MOTOR_INDEX::FLHR_IDX]);
    ui->QCP_MT_POS_FLHP->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredPosition[MOTOR_INDEX::FLHP_IDX]);
    ui->QCP_MT_POS_FLKP->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredPosition[MOTOR_INDEX::FLKP_IDX]);
    ui->QCP_MT_POS_FRHR->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredPosition[MOTOR_INDEX::FRHR_IDX]);
    ui->QCP_MT_POS_FRHP->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredPosition[MOTOR_INDEX::FRHP_IDX]);
    ui->QCP_MT_POS_FRKP->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredPosition[MOTOR_INDEX::FRKP_IDX]);
    ui->QCP_MT_POS_HLHR->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredPosition[MOTOR_INDEX::HLHR_IDX]);
    ui->QCP_MT_POS_HLHP->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredPosition[MOTOR_INDEX::HLHP_IDX]);
    ui->QCP_MT_POS_HLKP->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredPosition[MOTOR_INDEX::HLKP_IDX]);
    ui->QCP_MT_POS_HRHR->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredPosition[MOTOR_INDEX::HRHR_IDX]);
    ui->QCP_MT_POS_HRHP->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredPosition[MOTOR_INDEX::HRHP_IDX]);
    ui->QCP_MT_POS_HRKP->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredPosition[MOTOR_INDEX::HRKP_IDX]);

    ui->QCP_MT_VEL_FLHR->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorVelocity[MOTOR_INDEX::FLHR_IDX]);
    ui->QCP_MT_VEL_FLHP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorVelocity[MOTOR_INDEX::FLHP_IDX]);
    ui->QCP_MT_VEL_FLKP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorVelocity[MOTOR_INDEX::FLKP_IDX]);
    ui->QCP_MT_VEL_FRHR->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorVelocity[MOTOR_INDEX::FRHR_IDX]);
    ui->QCP_MT_VEL_FRHP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorVelocity[MOTOR_INDEX::FRHP_IDX]);
    ui->QCP_MT_VEL_FRKP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorVelocity[MOTOR_INDEX::FRKP_IDX]);
    ui->QCP_MT_VEL_HLHR->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorVelocity[MOTOR_INDEX::HLHR_IDX]);
    ui->QCP_MT_VEL_HLHP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorVelocity[MOTOR_INDEX::HLHP_IDX]);
    ui->QCP_MT_VEL_HLKP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorVelocity[MOTOR_INDEX::HLKP_IDX]);
    ui->QCP_MT_VEL_HRHR->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorVelocity[MOTOR_INDEX::HRHR_IDX]);
    ui->QCP_MT_VEL_HRHP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorVelocity[MOTOR_INDEX::HRHP_IDX]);
    ui->QCP_MT_VEL_HRKP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorVelocity[MOTOR_INDEX::HRKP_IDX]);
    ui->QCP_MT_VEL_FLHR->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredVelocity[MOTOR_INDEX::FLHR_IDX]);
    ui->QCP_MT_VEL_FLHP->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredVelocity[MOTOR_INDEX::FLHP_IDX]);
    ui->QCP_MT_VEL_FLKP->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredVelocity[MOTOR_INDEX::FLKP_IDX]);
    ui->QCP_MT_VEL_FRHR->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredVelocity[MOTOR_INDEX::FRHR_IDX]);
    ui->QCP_MT_VEL_FRHP->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredVelocity[MOTOR_INDEX::FRHP_IDX]);
    ui->QCP_MT_VEL_FRKP->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredVelocity[MOTOR_INDEX::FRKP_IDX]);
    ui->QCP_MT_VEL_HLHR->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredVelocity[MOTOR_INDEX::HLHR_IDX]);
    ui->QCP_MT_VEL_HLHP->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredVelocity[MOTOR_INDEX::HLHP_IDX]);
    ui->QCP_MT_VEL_HLKP->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredVelocity[MOTOR_INDEX::HLKP_IDX]);
    ui->QCP_MT_VEL_HRHR->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredVelocity[MOTOR_INDEX::HRHR_IDX]);
    ui->QCP_MT_VEL_HRHP->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredVelocity[MOTOR_INDEX::HRHP_IDX]);
    ui->QCP_MT_VEL_HRKP->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredVelocity[MOTOR_INDEX::HRKP_IDX]);

    ui->QCP_MT_TAU_FLHR->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorTorque[MOTOR_INDEX::FLHR_IDX]);
    ui->QCP_MT_TAU_FLHP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorTorque[MOTOR_INDEX::FLHP_IDX]);
    ui->QCP_MT_TAU_FLKP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorTorque[MOTOR_INDEX::FLKP_IDX]);
    ui->QCP_MT_TAU_FRHR->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorTorque[MOTOR_INDEX::FRHR_IDX]);
    ui->QCP_MT_TAU_FRHP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorTorque[MOTOR_INDEX::FRHP_IDX]);
    ui->QCP_MT_TAU_FRKP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorTorque[MOTOR_INDEX::FRKP_IDX]);
    ui->QCP_MT_TAU_HLHR->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorTorque[MOTOR_INDEX::HLHR_IDX]);
    ui->QCP_MT_TAU_HLHP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorTorque[MOTOR_INDEX::HLHP_IDX]);
    ui->QCP_MT_TAU_HLKP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorTorque[MOTOR_INDEX::HLKP_IDX]);
    ui->QCP_MT_TAU_HRHR->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorTorque[MOTOR_INDEX::HRHR_IDX]);
    ui->QCP_MT_TAU_HRHP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorTorque[MOTOR_INDEX::HRHP_IDX]);
    ui->QCP_MT_TAU_HRKP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorTorque[MOTOR_INDEX::HRKP_IDX]);
    ui->QCP_MT_TAU_FLHR->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredTorque[MOTOR_INDEX::FLHR_IDX]);
    ui->QCP_MT_TAU_FLHP->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredTorque[MOTOR_INDEX::FLHP_IDX]);
    ui->QCP_MT_TAU_FLKP->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredTorque[MOTOR_INDEX::FLKP_IDX]);
    ui->QCP_MT_TAU_FRHR->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredTorque[MOTOR_INDEX::FRHR_IDX]);
    ui->QCP_MT_TAU_FRHP->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredTorque[MOTOR_INDEX::FRHP_IDX]);
    ui->QCP_MT_TAU_FRKP->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredTorque[MOTOR_INDEX::FRKP_IDX]);
    ui->QCP_MT_TAU_HLHR->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredTorque[MOTOR_INDEX::HLHR_IDX]);
    ui->QCP_MT_TAU_HLHP->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredTorque[MOTOR_INDEX::HLHP_IDX]);
    ui->QCP_MT_TAU_HLKP->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredTorque[MOTOR_INDEX::HLKP_IDX]);
    ui->QCP_MT_TAU_HRHR->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredTorque[MOTOR_INDEX::HRHR_IDX]);
    ui->QCP_MT_TAU_HRHP->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredTorque[MOTOR_INDEX::HRHP_IDX]);
    ui->QCP_MT_TAU_HRKP->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredTorque[MOTOR_INDEX::HRKP_IDX]);
    ui->QCP_CONTACT_FL->graph(0)->addData(sharedMemory->localTime, (double)sharedMemory->contactState[0]);
    ui->QCP_CONTACT_FR->graph(0)->addData(sharedMemory->localTime, (double)sharedMemory->contactState[1]);
    ui->QCP_CONTACT_HL->graph(0)->addData(sharedMemory->localTime, (double)sharedMemory->contactState[2]);
    ui->QCP_CONTACT_HR->graph(0)->addData(sharedMemory->localTime, (double)sharedMemory->contactState[3]);
    ui->QCP_CONTACT_FL->graph(1)->addData(sharedMemory->localTime, (double)sharedMemory->gaitTable[0]);
    ui->QCP_CONTACT_FR->graph(1)->addData(sharedMemory->localTime, (double)sharedMemory->gaitTable[1]);
    ui->QCP_CONTACT_HL->graph(1)->addData(sharedMemory->localTime, (double)sharedMemory->gaitTable[2]);
    ui->QCP_CONTACT_HR->graph(1)->addData(sharedMemory->localTime, (double)sharedMemory->gaitTable[3]);
    ui->QCP_CONTACT_FL->graph(2)->addData(sharedMemory->localTime, (double)sharedMemory->contactResidualTorque[0]);
    ui->QCP_CONTACT_FR->graph(2)->addData(sharedMemory->localTime, (double)sharedMemory->contactResidualTorque[1]);
    ui->QCP_CONTACT_HL->graph(2)->addData(sharedMemory->localTime, (double)sharedMemory->contactResidualTorque[2]);
    ui->QCP_CONTACT_HR->graph(2)->addData(sharedMemory->localTime, (double)sharedMemory->contactResidualTorque[3]);
}

void MainWindow::DeveloperDisplayUpdate()
{
    if (ui->CB_DEVELOPER_MODE->isChecked())
    {
        addGraphData();
        if (ui->TAB_CONTROL->isVisible())
        {
            if (mIsUpdateGraph)
            {
                if (ui->TAB_LINEAR->isVisible())
                {
                    ui->QCP_BODY_POS_X->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                    ui->QCP_BODY_POS_X->yAxis->setRange(sharedMemory->globalBaseDesiredPosition[0] - 2.5, sharedMemory->globalBaseDesiredPosition[0] + 2.5);
                    ui->QCP_BODY_POS_Y->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                    ui->QCP_BODY_POS_Y->yAxis->setRange(sharedMemory->globalBaseDesiredPosition[1] - 2.5, sharedMemory->globalBaseDesiredPosition[1] + 2.5);
                    ui->QCP_BODY_POS_Z->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                    ui->QCP_BODY_POS_Z->yAxis->setRange(sharedMemory->globalBaseDesiredPosition[2] - 0.05, sharedMemory->globalBaseDesiredPosition[2] + 0.05);
                    ui->QCP_BODY_VEL_X->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                    ui->QCP_BODY_VEL_Y->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                    ui->QCP_BODY_VEL_Z->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                    ui->QCP_BODY_POS_X->replot();
                    ui->QCP_BODY_POS_Y->replot();
                    ui->QCP_BODY_POS_Z->replot();
                    ui->QCP_BODY_VEL_X->replot();
                    ui->QCP_BODY_VEL_Y->replot();
                    ui->QCP_BODY_VEL_Z->replot();
                }
                else if (ui->TAB_ANGULAR->isVisible())
                {
                    ui->QCP_BODY_ANG_POS_X->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                    ui->QCP_BODY_ANG_POS_Y->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                    ui->QCP_BODY_ANG_POS_Z->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                    ui->QCP_BODY_ANG_POS_Z->yAxis->setRange(sharedMemory->globalBaseDesiredEulerAngle[2] - 1.0, sharedMemory->globalBaseDesiredEulerAngle[2] + 1.0);
                    ui->QCP_BODY_ANG_VEL_X->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                    ui->QCP_BODY_ANG_VEL_Y->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                    ui->QCP_BODY_ANG_VEL_Z->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);

                    ui->QCP_BODY_ANG_POS_X->replot();
                    ui->QCP_BODY_ANG_POS_Y->replot();
                    ui->QCP_BODY_ANG_POS_Z->replot();
                    ui->QCP_BODY_ANG_VEL_X->replot();
                    ui->QCP_BODY_ANG_VEL_Y->replot();
                    ui->QCP_BODY_ANG_VEL_Z->replot();
                }
                else if (ui->TAB_MOTOR->isVisible())
                {
                    if (ui->tab_motor_position->isVisible())
                    {
                        ui->QCP_MT_POS_FLHR->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_POS_FLHP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_POS_FLKP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_POS_FRHR->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_POS_FRHP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_POS_FRKP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_POS_HLHR->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_POS_HLHP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_POS_HLKP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_POS_HRHR->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_POS_HRHP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_POS_HRKP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_POS_FLHR->replot();
                        ui->QCP_MT_POS_FLHP->replot();
                        ui->QCP_MT_POS_FLKP->replot();
                        ui->QCP_MT_POS_FRHR->replot();
                        ui->QCP_MT_POS_FRHP->replot();
                        ui->QCP_MT_POS_FRKP->replot();
                        ui->QCP_MT_POS_HLHR->replot();
                        ui->QCP_MT_POS_HLHP->replot();
                        ui->QCP_MT_POS_HLKP->replot();
                        ui->QCP_MT_POS_HRHR->replot();
                        ui->QCP_MT_POS_HRHP->replot();
                        ui->QCP_MT_POS_HRKP->replot();
                    }
                    else if (ui->tab_motor_velocity->isVisible())
                    {
                        ui->QCP_MT_VEL_FLHR->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_VEL_FLHP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_VEL_FLKP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_VEL_FRHR->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_VEL_FRHP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_VEL_FRKP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_VEL_HLHR->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_VEL_HLHP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_VEL_HLKP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_VEL_HRHR->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_VEL_HRHP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_VEL_HRKP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_VEL_FLHR->replot();
                        ui->QCP_MT_VEL_FLHP->replot();
                        ui->QCP_MT_VEL_FLKP->replot();
                        ui->QCP_MT_VEL_FRHR->replot();
                        ui->QCP_MT_VEL_FRHP->replot();
                        ui->QCP_MT_VEL_FRKP->replot();
                        ui->QCP_MT_VEL_HLHR->replot();
                        ui->QCP_MT_VEL_HLHP->replot();
                        ui->QCP_MT_VEL_HLKP->replot();
                        ui->QCP_MT_VEL_HRHR->replot();
                        ui->QCP_MT_VEL_HRHP->replot();
                        ui->QCP_MT_VEL_HRKP->replot();
                    }
                    else if (ui->tab_motor_torque->isVisible())
                    {
                        ui->QCP_MT_TAU_FLHR->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_TAU_FLHP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_TAU_FLKP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_TAU_FRHR->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_TAU_FRHP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_TAU_FRKP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_TAU_HLHR->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_TAU_HLHP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_TAU_HLKP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_TAU_HRHR->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_TAU_HRHP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_TAU_HRKP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                        ui->QCP_MT_TAU_FLHR->replot();
                        ui->QCP_MT_TAU_FLHP->replot();
                        ui->QCP_MT_TAU_FLKP->replot();
                        ui->QCP_MT_TAU_FRHR->replot();
                        ui->QCP_MT_TAU_FRHP->replot();
                        ui->QCP_MT_TAU_FRKP->replot();
                        ui->QCP_MT_TAU_HLHR->replot();
                        ui->QCP_MT_TAU_HLHP->replot();
                        ui->QCP_MT_TAU_HLKP->replot();
                        ui->QCP_MT_TAU_HRHR->replot();
                        ui->QCP_MT_TAU_HRHP->replot();
                        ui->QCP_MT_TAU_HRKP->replot();
                    }
                }
                else if (ui->TAB_CONTACT->isVisible())
                {
                    ui->QCP_CONTACT_FL->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                    ui->QCP_CONTACT_FR->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                    ui->QCP_CONTACT_HL->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                    ui->QCP_CONTACT_HR->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
                    ui->QCP_CONTACT_FL->replot();
                    ui->QCP_CONTACT_FR->replot();
                    ui->QCP_CONTACT_HL->replot();
                    ui->QCP_CONTACT_HR->replot();
                }
            }

        }
        else if (ui->TAB_CONSOLE->isVisible())
        {
            ui->LE_CONSOLE_NAME_2->setText(QString(QString::fromStdString(sharedMemory->gamepad.controllerName))
            );
            if (sharedMemory->gamepad.button.Start)
            {
                ui->LE_JOYSTICK_START->setStyleSheet("background-color:lightblue");
            }
            else
            {
                ui->LE_JOYSTICK_START->setStyleSheet("background-color:gray");
            }
            if (sharedMemory->gamepad.button.A)
            {
                ui->LE_JOYSTICK_A->setStyleSheet("background-color:lightblue");
            }
            else
            {
                ui->LE_JOYSTICK_A->setStyleSheet("background-color:gray");
            }
            if (sharedMemory->gamepad.button.B)
            {
                ui->LE_JOYSTICK_B->setStyleSheet("background-color:lightblue");
            }
            else
            {
                ui->LE_JOYSTICK_B->setStyleSheet("background-color:gray");
            }
            if (sharedMemory->gamepad.button.X)
            {
                ui->LE_JOYSTICK_X->setStyleSheet("background-color:lightblue");
            }
            else
            {
                ui->LE_JOYSTICK_X->setStyleSheet("background-color:gray");
            }
            if (sharedMemory->gamepad.button.Y)
            {
                ui->LE_JOYSTICK_Y->setStyleSheet("background-color:lightblue");
            }
            else
            {
                ui->LE_JOYSTICK_Y->setStyleSheet("background-color:gray");
            }
            if (sharedMemory->gamepad.button.LB)
            {
                ui->LE_JOYSTICK_LB->setStyleSheet("background-color:lightblue");
            }
            else
            {
                ui->LE_JOYSTICK_LB->setStyleSheet("background-color:gray");
            }
            if (sharedMemory->gamepad.button.RB)
            {
                ui->LE_JOYSTICK_RB->setStyleSheet("background-color:lightblue");
            }
            else
            {
                ui->LE_JOYSTICK_RB->setStyleSheet("background-color:gray");
            }
            if (sharedMemory->gamepad.joystick.LeftTrigger > 0.5)
            {
                ui->LE_JOYSTICK_LT->setStyleSheet("background-color:lightblue");
            }
            else
            {
                ui->LE_JOYSTICK_LT->setStyleSheet("background-color:gray");
            }
            if (sharedMemory->gamepad.joystick.RightTrigger > 0.5)
            {
                ui->LE_JOYSTICK_RT->setStyleSheet("background-color:lightblue");
            }
            else
            {
                ui->LE_JOYSTICK_RT->setStyleSheet("background-color:gray");
            }
            if (sharedMemory->gamepad.joystick.DpadX < 0)
            {
                ui->LE_JOYSTICK_LEFT->setStyleSheet("background-color:lightblue");
            }
            else
            {
                ui->LE_JOYSTICK_LEFT->setStyleSheet("background-color:gray");
            }
            if (sharedMemory->gamepad.joystick.DpadX > 0)
            {
                ui->LE_JOYSTICK_RIGHT->setStyleSheet("background-color:lightblue");
            }
            else
            {
                ui->LE_JOYSTICK_RIGHT->setStyleSheet("background-color:gray");
            }
            if (sharedMemory->gamepad.joystick.DpadY > 0)
            {
                ui->LE_JOYSTICK_UP->setStyleSheet("background-color:lightblue");
            }
            else
            {
                ui->LE_JOYSTICK_UP->setStyleSheet("background-color:gray");
            }
            if (sharedMemory->gamepad.joystick.DpadY < 0)
            {
                ui->LE_JOYSTICK_DOWN->setStyleSheet("background-color:lightblue");
            }
            else
            {
                ui->LE_JOYSTICK_DOWN->setStyleSheet("background-color:gray");
            }
            ui->LE_JOYSTICK_LEFT_X->setText(QString().sprintf("%.2f", sharedMemory->gamepad.joystick.LeftStickX));
            ui->LE_JOYSTICK_LEFT_Y->setText(QString().sprintf("%.2f", sharedMemory->gamepad.joystick.LeftStickY));
            ui->LE_JOYSTICK_RIGHT_X->setText(QString().sprintf("%.2f", sharedMemory->gamepad.joystick.RightStickX));
            ui->LE_JOYSTICK_RIGHT_Y->setText(QString().sprintf("%.2f", sharedMemory->gamepad.joystick.RightStickY));

            ui->QCP_JOYSTICK_LEFT->graph(0)->data()->clear();
            ui->QCP_JOYSTICK_RIGHT->graph(0)->data()->clear();
            ui->QCP_JOYSTICK_LEFT->graph(0)->addData(sharedMemory->gamepad.joystick.LeftStickX, sharedMemory->gamepad.joystick.LeftStickY);
            ui->QCP_JOYSTICK_RIGHT->graph(0)->addData(sharedMemory->gamepad.joystick.RightStickX, sharedMemory->gamepad.joystick.RightStickY);
            ui->QCP_JOYSTICK_LEFT->replot();
            ui->QCP_JOYSTICK_RIGHT->replot();
        }
    }
}

void MainWindow::checkPrevData()
{
    static int cnt;
    if (sharedMemory->gamepad.gui.GUIButton != GUI_NO_ACT)
    {
        cnt++;
    }
    if(cnt >= 50){
        sharedMemory->gamepad.gui.GUIButton = GUI_NO_ACT;
        cnt = 0;
    }
    if (mPrevDeveloperModeChecked != ui->CB_DEVELOPER_MODE->isChecked())
    {
        mPrevDeveloperModeChecked = ui->CB_DEVELOPER_MODE->isChecked();
        if (mPrevDeveloperModeChecked)
        {
            ui->GB_GRAPH_OPTION->show();
        }
        else
        {
            ui->GB_GRAPH_OPTION->hide();
        }
    }
    if (mPrevTCPConnected != sharedMemory->isTCPConnected)
    {
        if (sharedMemory->isTCPConnected)
        {
            ui->LE_CONNECT_1->setStyleSheet("background-color:lightgreen");
            ui->LE_CONNECT_2->setStyleSheet("background-color:lightgreen");
            mNewGUIinputSignal = true;
        }
        else
        {
            ui->LE_ROBOT_STATE->setText("DISCONNECTED");
            ui->LE_CONNECT_1->setStyleSheet("background-color:red");
            ui->LE_CONNECT_2->setStyleSheet("background-color:red");
        }
        mPrevTCPConnected = sharedMemory->isTCPConnected;
    }

    if (mPrevFSMState != sharedMemory->FSMState)
    {
        sharedMemory->gamepad.gui.GUIButton = GUI_NO_ACT;
        mPrevFSMState = sharedMemory->FSMState;
        mNewGUIinputSignal = true;
    }
}

void MainWindow::on_BT_GRAPH_RESET_clicked()
{
    clearGraph(ui->QCP_BODY_POS_X);
    clearGraph(ui->QCP_BODY_VEL_X);
    clearGraph(ui->QCP_BODY_POS_Y);
    clearGraph(ui->QCP_BODY_VEL_Y);
    clearGraph(ui->QCP_BODY_POS_Z);
    clearGraph(ui->QCP_BODY_VEL_Z);
    clearGraph(ui->QCP_BODY_ANG_POS_X);
    clearGraph(ui->QCP_BODY_ANG_POS_Y);
    clearGraph(ui->QCP_BODY_ANG_POS_Z);
    clearGraph(ui->QCP_BODY_ANG_VEL_X);
    clearGraph(ui->QCP_BODY_ANG_VEL_Y);
    clearGraph(ui->QCP_BODY_ANG_VEL_Z);
    clearGraph(ui->QCP_JOYSTICK_LEFT);
    clearGraph(ui->QCP_JOYSTICK_RIGHT);
    clearGraph(ui->QCP_MT_POS_FLHP);
    clearGraph(ui->QCP_MT_POS_HLHR);
    clearGraph(ui->QCP_MT_POS_FRHP);
    clearGraph(ui->QCP_MT_POS_FLHR);
    clearGraph(ui->QCP_MT_POS_FRHR);
    clearGraph(ui->QCP_MT_POS_HLHP);
    clearGraph(ui->QCP_MT_POS_FRKP);
    clearGraph(ui->QCP_MT_POS_FLKP);
    clearGraph(ui->QCP_MT_POS_HLKP);
    clearGraph(ui->QCP_MT_POS_HRKP);
    clearGraph(ui->QCP_MT_POS_HRHP);
    clearGraph(ui->QCP_MT_POS_HRHR);
    clearGraph(ui->QCP_MT_VEL_FRHP);
    clearGraph(ui->QCP_MT_VEL_HLKP);
    clearGraph(ui->QCP_MT_VEL_HRKP);
    clearGraph(ui->QCP_MT_VEL_FLHP);
    clearGraph(ui->QCP_MT_VEL_FRHR);
    clearGraph(ui->QCP_MT_VEL_HRHR);
    clearGraph(ui->QCP_MT_VEL_HRHP);
    clearGraph(ui->QCP_MT_VEL_FRKP);
    clearGraph(ui->QCP_MT_VEL_FLKP);
    clearGraph(ui->QCP_MT_VEL_HLHP);
    clearGraph(ui->QCP_MT_VEL_HLHR);
    clearGraph(ui->QCP_MT_VEL_FLHR);
    clearGraph(ui->QCP_MT_TAU_FRHP);
    clearGraph(ui->QCP_MT_TAU_HLHP);
    clearGraph(ui->QCP_MT_TAU_HRHP);
    clearGraph(ui->QCP_MT_TAU_HRKP);
    clearGraph(ui->QCP_MT_TAU_HLKP);
    clearGraph(ui->QCP_MT_TAU_FLHP);
    clearGraph(ui->QCP_MT_TAU_FRKP);
    clearGraph(ui->QCP_MT_TAU_HLHR);
    clearGraph(ui->QCP_MT_TAU_FRHR);
    clearGraph(ui->QCP_MT_TAU_FLHR);
    clearGraph(ui->QCP_MT_TAU_FLKP);
    clearGraph(ui->QCP_MT_TAU_HRHR);
    clearGraph(ui->QCP_CONTACT_FR);
    clearGraph(ui->QCP_CONTACT_HR);
    clearGraph(ui->QCP_CONTACT_FL);
    clearGraph(ui->QCP_CONTACT_HL);
}

void MainWindow::clearGraph(QCustomPlot* QCP)
{
    for (int i = 0; i < QCP->graphCount(); i++)
    {
        QCP->graph(i)->data()->clear();
    }
    QCP->replot();
}

void MainWindow::DisplayUpdate()
{
    checkPrevData();
    if(sharedMemory->stumbleRecovery)
    {
        ui->LE_STUMBLE->setStyleSheet("background-color:lightgreen");
    }
    else
    {
        ui->LE_STUMBLE->setStyleSheet("background-color:pink");
    }
    if (mNewGUIinputSignal)
    {
        switch (sharedMemory->FSMState)
        {
        case FSM_INITIAL:
            on_BT_GRAPH_RESET_clicked();
            ui->LE_ROBOT_STATE->setText("INITIAL");
            ui->BT_ROBOT_START->setEnabled(true);
            ui->BT_STAND_UP->setDisabled(true);
            ui->BT_SIT_DOWN->setDisabled(true);
            ui->BT_TROT_SLOW->setDisabled(true);
            ui->BT_TROT_FAST->setDisabled(true);
            ui->BT_TROT_OVERLAP->setDisabled(true);
            ui->BT_TROT_STOP->setDisabled(true);
            ui->BT_E_STOP->setDisabled(true);
            ui->BT_ROBOT_RESTART->setDisabled(true);

            ui->LE_TROT_OVERLAP->setStyleSheet("background-color:pink");
            ui->LE_TROT_SLOW->setStyleSheet("background-color:pink");
            ui->LE_TROT_FAST->setStyleSheet("background-color:pink");
            ui->LE_TROT_STOP->setStyleSheet("background-color:pink");
            break;
        case FSM_EMERGENCY_STOP:
            ui->LE_ROBOT_STATE->setText("E-STOP");
            ui->BT_STAND_UP->setDisabled(true);
            ui->BT_SIT_DOWN->setDisabled(true);
            ui->BT_TROT_SLOW->setDisabled(true);
            ui->BT_TROT_FAST->setDisabled(true);
            ui->BT_TROT_OVERLAP->setDisabled(true);
            ui->BT_TROT_STOP->setDisabled(true);
            ui->BT_ROBOT_START->setDisabled(true);
            ui->BT_ROBOT_RESTART->setEnabled(true);
            ui->LE_TROT_OVERLAP->setStyleSheet("background-color:pink");
            ui->LE_TROT_SLOW->setStyleSheet("background-color:pink");
            ui->LE_TROT_FAST->setStyleSheet("background-color:pink");
            ui->LE_TROT_STOP->setStyleSheet("background-color:pink");
            break;
        case FSM_READY:
            ui->LE_ROBOT_STATE->setText("READY");
            ui->BT_ROBOT_START->setDisabled(true);
            ui->BT_STAND_UP->setEnabled(true);
            ui->BT_E_STOP->setEnabled(true);
            ui->BT_SIT_DOWN->setDisabled(true);
            ui->BT_TROT_SLOW->setDisabled(true);
            ui->BT_TROT_FAST->setDisabled(true);
            ui->BT_TROT_OVERLAP->setDisabled(true);
            ui->BT_TROT_STOP->setDisabled(true);
            ui->LE_TROT_OVERLAP->setStyleSheet("background-color:pink");
            ui->LE_TROT_SLOW->setStyleSheet("background-color:pink");
            ui->LE_TROT_FAST->setStyleSheet("background-color:pink");
            ui->LE_TROT_STOP->setStyleSheet("background-color:pink");

            break;
        case FSM_STAND_UP:
            ui->LE_ROBOT_STATE->setText("STAND-UP");
            ui->BT_E_STOP->setEnabled(true);
            ui->BT_ROBOT_START->setDisabled(true);
            ui->BT_STAND_UP->setDisabled(true);
            ui->BT_SIT_DOWN->setDisabled(true);
            ui->BT_TROT_SLOW->setDisabled(true);
            ui->BT_TROT_FAST->setDisabled(true);
            ui->BT_TROT_OVERLAP->setDisabled(true);
            ui->BT_TROT_STOP->setDisabled(true);
            ui->LE_TROT_OVERLAP->setStyleSheet("background-color:pink");
            ui->LE_TROT_SLOW->setStyleSheet("background-color:pink");
            ui->LE_TROT_FAST->setStyleSheet("background-color:pink");
            ui->LE_TROT_STOP->setStyleSheet("background-color:pink");
            break;
        case FSM_SIT_DOWN:
            ui->LE_ROBOT_STATE->setText("SIT-DOWN");
            ui->BT_E_STOP->setEnabled(true);
            ui->BT_ROBOT_START->setDisabled(true);
            ui->BT_STAND_UP->setDisabled(true);
            ui->BT_SIT_DOWN->setDisabled(true);
            ui->BT_TROT_SLOW->setDisabled(true);
            ui->BT_TROT_FAST->setDisabled(true);
            ui->BT_TROT_OVERLAP->setDisabled(true);
            ui->BT_TROT_STOP->setDisabled(true);
            ui->LE_TROT_OVERLAP->setStyleSheet("background-color:pink");
            ui->LE_TROT_SLOW->setStyleSheet("background-color:pink");
            ui->LE_TROT_FAST->setStyleSheet("background-color:pink");
            ui->LE_TROT_STOP->setStyleSheet("background-color:pink");
            break;
        case FSM_STAND:
            ui->LE_ROBOT_STATE->setText("STAND");
            ui->BT_E_STOP->setEnabled(true);
            ui->BT_ROBOT_START->setDisabled(true);
            ui->BT_STAND_UP->setDisabled(true);
            ui->BT_SIT_DOWN->setEnabled(true);
            ui->BT_TROT_SLOW->setEnabled(true);
            ui->BT_TROT_FAST->setEnabled(true);
            ui->BT_TROT_OVERLAP->setEnabled(true);
            ui->BT_TROT_STOP->setEnabled(true);
            ui->LE_TROT_OVERLAP->setStyleSheet("background-color:pink");
            ui->LE_TROT_SLOW->setStyleSheet("background-color:pink");
            ui->LE_TROT_FAST->setStyleSheet("background-color:pink");
            ui->LE_TROT_STOP->setStyleSheet("background-color:lightgreen");
            break;
        case FSM_TROT_STOP:
            ui->LE_ROBOT_STATE->setText("STOPPING");
            ui->BT_E_STOP->setEnabled(true);
            ui->BT_ROBOT_START->setDisabled(true);
            ui->BT_STAND_UP->setDisabled(true);
            ui->BT_SIT_DOWN->setDisabled(true);
            ui->BT_TROT_SLOW->setDisabled(true);
            ui->BT_TROT_FAST->setDisabled(true);
            ui->BT_TROT_OVERLAP->setDisabled(true);
            ui->BT_TROT_STOP->setEnabled(true);
            ui->LE_TROT_OVERLAP->setStyleSheet("background-color:pink");
            ui->LE_TROT_SLOW->setStyleSheet("background-color:pink");
            ui->LE_TROT_FAST->setStyleSheet("background-color:pink");
            ui->LE_TROT_STOP->setStyleSheet("background-color:lightgreen");
            break;
        case FSM_TROT_SLOW:
            ui->LE_ROBOT_STATE->setText("TROT-SLOW");
            ui->BT_E_STOP->setEnabled(true);
            ui->BT_TROT_STOP->setEnabled(true);
            ui->BT_TROT_FAST->setEnabled(true);
            ui->BT_TROT_OVERLAP->setEnabled(true);
            ui->BT_TROT_SLOW->setEnabled(true);
            ui->BT_ROBOT_START->setDisabled(true);
            ui->BT_SIT_DOWN->setDisabled(true);
            ui->LE_TROT_SLOW->setStyleSheet("background-color:lightgreen");
            ui->LE_TROT_OVERLAP->setStyleSheet("background-color:pink");
            ui->LE_TROT_FAST->setStyleSheet("background-color:pink");
            ui->LE_TROT_STOP->setStyleSheet("background-color:pink");
            break;
        case FSM_TROT_FAST:
            ui->LE_ROBOT_STATE->setText("TROT-FAST");
            ui->BT_E_STOP->setEnabled(true);
            ui->BT_TROT_STOP->setEnabled(true);
            ui->BT_TROT_FAST->setEnabled(true);
            ui->BT_TROT_OVERLAP->setEnabled(true);
            ui->BT_TROT_SLOW->setEnabled(true);
            ui->BT_ROBOT_START->setDisabled(true);
            ui->BT_SIT_DOWN->setDisabled(true);
            ui->LE_TROT_OVERLAP->setStyleSheet("background-color:pink");
            ui->LE_TROT_SLOW->setStyleSheet("background-color:pink");
            ui->LE_TROT_FAST->setStyleSheet("background-color:lightgreen");
            ui->LE_TROT_STOP->setStyleSheet("background-color:pink");
            break;
        case FSM_OVERLAP_TROT_FAST:
            ui->LE_ROBOT_STATE->setText("TROT-OVERLAP");
            ui->BT_E_STOP->setEnabled(true);
            ui->BT_TROT_STOP->setEnabled(true);
            ui->BT_TROT_FAST->setEnabled(true);
            ui->BT_TROT_OVERLAP->setEnabled(true);
            ui->BT_TROT_SLOW->setEnabled(true);
            ui->BT_ROBOT_START->setDisabled(true);
            ui->BT_SIT_DOWN->setDisabled(true);
            ui->LE_TROT_OVERLAP->setStyleSheet("background-color:lightgreen");
            ui->LE_TROT_SLOW->setStyleSheet("background-color:pink");
            ui->LE_TROT_FAST->setStyleSheet("background-color:pink");
            ui->LE_TROT_STOP->setStyleSheet("background-color:pink");
            break;
        case FSM_RESTART:
            /// no use this FSM
            ui->BT_ROBOT_RESTART->setDisabled(true);
            ui->LE_ROBOT_STATE->setText("RESTART");
            break;
        default:
            ui->LE_ROBOT_STATE->setText("ERROR");
            ui->BT_E_STOP->setEnabled(true);
            break;
        }
        mNewGUIinputSignal = false;
    }

    if (ui->TAB_ROBOT->isVisible())
    {
//        ui->TM_LOG->appendPlainText(QString::fromUtf8("hello"));
        ui->TW_IMU_RPY->setItem(0, 0, new QTableWidgetItem(QString::number((int)(sharedMemory->globalBaseEulerAngle[0] * R2D))));
        ui->TW_IMU_RPY->item(0, 0)->setTextAlignment(Qt::AlignCenter);

        ui->TW_IMU_RPY->setItem(0, 1, new QTableWidgetItem(QString::number((int)(sharedMemory->globalBaseEulerAngle[1] * R2D))));
        ui->TW_IMU_RPY->item(0, 1)->
            setTextAlignment(Qt::AlignCenter);

        ui->TW_IMU_RPY->setItem(0, 2, new QTableWidgetItem(QString::number((int)(sharedMemory->globalBaseEulerAngle[2] * R2D))));
        ui->TW_IMU_RPY->item(0, 2)->setTextAlignment(Qt::AlignCenter);

        ui->TW_IMU_GYRO->setItem(0, 0, new QTableWidgetItem(QString::number(sharedMemory->bodyBaseAngularVelocity[0], 'f', 3)));
        ui->TW_IMU_GYRO->item(0, 0)->setTextAlignment(Qt::AlignCenter);

        ui->TW_IMU_GYRO->setItem(0, 1, new QTableWidgetItem(QString::number(sharedMemory->bodyBaseAngularVelocity[1], 'f', 3)));
        ui->TW_IMU_GYRO->item(0, 1)->setTextAlignment(Qt::AlignCenter);

        ui->TW_IMU_GYRO->setItem(0, 2, new QTableWidgetItem(QString::number(sharedMemory->bodyBaseAngularVelocity[2], 'f', 3)));
        ui->TW_IMU_GYRO->item(0, 2)->setTextAlignment(Qt::AlignCenter);

        ui->TW_IMU_ACC->setItem(0, 0, new QTableWidgetItem(QString::number(sharedMemory->globalBaseAcceleration[0], 'f', 3)));
        ui->TW_IMU_ACC->item(0, 0)->setTextAlignment(Qt::AlignCenter);

        ui->TW_IMU_ACC->setItem(0, 1, new QTableWidgetItem(QString::number(sharedMemory->globalBaseAcceleration[1], 'f', 3)));
        ui->TW_IMU_ACC->item(0, 1)->setTextAlignment(Qt::AlignCenter);

        ui->TW_IMU_ACC->setItem(0, 2, new QTableWidgetItem(QString::number(sharedMemory->globalBaseAcceleration[2], 'f', 3)));
        ui->TW_IMU_ACC->item(0, 2)->setTextAlignment(Qt::AlignCenter);

        for (int index = 0; index < 12; index++)
        {
            /// visualize motor on/off
            if (sharedMemory->motorStatus)
            {
                ui->TB_MOTOR->verticalHeaderItem(index)->setBackground(QColor(75, 75, 255));
//                ui->TB_MOTOR->item(index, 0)->setBackgroundColor(QColor(75, 75, 255));
            }
            else
            {
                ui->TB_MOTOR->verticalHeaderItem(index)->setBackground(QColor(255, 75, 75));
//                ui->TB_MOTOR->item(index, 0)->setBackgroundColor(QColor(255, 75, 75));
            }

            /// visualize motor error state
            ui->TB_MOTOR->setItem(index, 0, new QTableWidgetItem(QString::number(sharedMemory->motorErrorStatus[index])));
            ui->TB_MOTOR->item(index, 0)->setTextAlignment(Qt::AlignCenter);
//            ui->TB_MOTOR->item(index, 1)->setText(QString().sprintf("%d", sharedMemory->motorErrorStatus[index]));
            if (sharedMemory->motorErrorStatus[index] != 0)
            {
                ui->TB_MOTOR->item(index, 0)->setBackgroundColor(QColor(255, 100, 100));
            }
            else
            {
                ui->TB_MOTOR->item(index, 0)->setBackgroundColor(QColor(100, 255, 100));
            }

            ui->TB_MOTOR->setItem(index, 1, new QTableWidgetItem(QString::number(sharedMemory->motorTemp[index])));
            ui->TB_MOTOR->item(index, 1)->setTextAlignment(Qt::AlignCenter);

            ui->TB_MOTOR->setItem(index, 2, new QTableWidgetItem(QString::number(sharedMemory->motorVoltage[index], 'f', 1)));
            ui->TB_MOTOR->item(index, 2)->setTextAlignment(Qt::AlignCenter);

            ui->TB_MOTOR->setItem(index, 3, new QTableWidgetItem(QString::number(sharedMemory->motorPosition[index] * R2D, 'f', 1)));
            ui->TB_MOTOR->item(index, 3)->setTextAlignment(Qt::AlignCenter);

            ui->TB_MOTOR->setItem(index, 4, new QTableWidgetItem(QString::number(sharedMemory->motorDesiredTorque[index], 'f', 1)));
            ui->TB_MOTOR->item(index, 4)->setTextAlignment(Qt::AlignCenter);

            ui->TB_MOTOR->setItem(index, 5, new QTableWidgetItem(QString::number(sharedMemory->motorTorque[index], 'f', 1)));
            ui->TB_MOTOR->item(index, 5)->setTextAlignment(Qt::AlignCenter);
        }
//        ui->LE_COMM_DELAY->
//            setText(QString("Delay : %1ms").arg(sharedMemory->TCP_Duration));

    }
}

void MainWindow::on_BT_ROBOT_START_clicked()
{
    sharedMemory->gamepad.gui.GUIButton = GUI_START;
}

void MainWindow::on_BT_E_STOP_clicked()
{
    sharedMemory->gamepad.gui.GUIButton = GUI_E_STOP;
}

void MainWindow::on_BT_GRAPH_DOWN_clicked()
{
    graphOffset *= 1.5;

}

void MainWindow::on_BT_GRAPH_PAUSE_GRAPH_clicked()
{
    mIsUpdateGraph = !mIsUpdateGraph;
}

void MainWindow::on_BT_GRAPH_UP_clicked()
{
    graphOffset /= 1.5;

}

void MainWindow::on_BT_STAND_UP_clicked()
{
    sharedMemory->gamepad.gui.GUIButton = GUI_HOME_UP;
}

void MainWindow::on_BT_SIT_DOWN_clicked()
{
    sharedMemory->gamepad.gui.GUIButton = GUI_HOME_DOWN;
}

void MainWindow::on_BT_TROT_OVERLAP_clicked()
{
    sharedMemory->gamepad.gui.GUIButton = GUI_TROT_OVERLAP;
}

void MainWindow::on_BT_TROT_SLOW_clicked()
{
    sharedMemory->gamepad.gui.GUIButton = GUI_TROT_SLOW;
}

void MainWindow::on_BT_TROT_FAST_clicked()
{
    sharedMemory->gamepad.gui.GUIButton = GUI_TROT_FAST;
}

void MainWindow::on_BT_TROT_STOP_clicked()
{
    sharedMemory->gamepad.gui.GUIButton = GUI_TROT_STOP;
}

void MainWindow::on_BT_IP_ADDRESS_clicked()
{
    /// is it needed???
    QString userInput = ui->TE_IP_ADDRESS->text();
    QRegExp ipRegex("^((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$");
    if (ipRegex.exactMatch(userInput))
    {
        // IP 주소 형식이 올바른 경우
        sharedMemory->robotAddress = userInput.toStdString();
        sharedMemory->isIPChanged = true;
        ui->LB_IP_ADDRESS->setText("<font color='green'>Valid IP address</font>");
    }
    else
    {
        // IP 주소 형식이 올바르지 않은 경우
        ui->LB_IP_ADDRESS->setText("<font color='red'>Invalid IP address</font>");
    }
}

void MainWindow::on_BT_ROBOT_RESTART_clicked()
{
    sharedMemory->gamepad.gui.GUIButton = GUI_RESTART;
}