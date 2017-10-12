#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "calibratesetting.h"
#include "calibratecam.h"

#include<QFileDialog>
#include<QTextStream>

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include <QMessageBox>

#include<opencv2/opencv.hpp>

using namespace cv;
using namespace std;


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

/*void MainWindow::appendPlainText(QString text){
   // ui->plainTextEdit->appendPlainText(text);
    //qDebug(text);
}*/

void MainWindow::on_actionCargar_Imagen_1_triggered()
{
    /*cv::Mat inputImage= cv::imread("/home/msuarez/Escritorio/1.jpg");
    cv::imshow("MainWindow",inputImage);*/

    QString nombre= QFileDialog::getOpenFileName();
    cv::Mat img1= cv::imread(nombre.toStdString());
    cv::namedWindow("Imagen 1",CV_WINDOW_AUTOSIZE);
    if(!img1.empty())
        cv::imshow("Imagen 1", img1);

        //img_1=img1;
    //img_1.show();
    //
    //cv::imshow("img_1",img1);
    qDebug("Anchura=%d,altura=%d",img1.cols,img1.rows);
    qDebug("Profundidad=%d,canales=%d",img1.depth(),img1.channels());
}

void MainWindow::on_actionCargar_Imagen_2_triggered()
{
    QString nombre= QFileDialog::getOpenFileName();
    cv::Mat img1= cv::imread(nombre.toStdString());
    cv::namedWindow("Imagen 2",CV_WINDOW_AUTOSIZE);
    if(!img1.empty())
        cv::imshow("Imagen 2", img1);

    qDebug("Anchura=%d,altura=%d",img1.cols,img1.rows);
    qDebug("Profundidad=%d,canales=%d",img1.depth(),img1.channels());
}

void MainWindow::on_actionSalir_triggered()
{
    this->close();
}



void MainWindow::on_actionCalibrar_camara_triggered()
{

    calibrateCam *calibCam = new calibrateCam;

    calibCam->show();

}

void MainWindow::on_actionStereo_Calibration_triggered()
{

    calibratesetting *calibrate = new calibratesetting;

    calibrate->show();

}
