#include "mainwindow.h"
#include "ui_mainwindow.h"

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

static bool readStringList( const string& filename, vector<string>& l )
{
    l.resize(0);
    FileStorage fs(filename, FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != FileNode::SEQ )
        return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
        l.push_back((string)*it);
    return true;
}

void PrintInfo(QString msg){

    QMessageBox msgBox;

    msgBox.setText(msg);
    msgBox.exec();

}

static QString StereoCalib(const vector<string>& imagelist, Size boardSize, bool useCalibrated=true, bool showRectified=true)
{
    QString info, error, archivo, number;

    //msg.append("can not open ").append(msg2).append(" or the string list is empty");


    if( imagelist.size() % 2 != 0 ) //informa error: no es un numero par de imagenes
    {
         error="Error: the image list contains odd (non-even) number of elements\n";
         PrintInfo(error);
        return error;
    }

    bool displayCorners = true;//false;
    const int maxScale = 2;
    const float squareSize = 1.f;  // Set this to your actual square size
    // ARRAY AND VECTOR STORAGE:

    vector<vector<Point2f> > imagePoints[2];
    vector<vector<Point3f> > objectPoints;
    Size imageSize;

    int i, j, k, nimages = (int)imagelist.size()/2;

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    vector<string> goodImageList;
    const string path="calibrate/";
    string pathfile="";

    for( i = j = 0; i < nimages; i++ ) //for #1
    {
        for( k = 0; k < 2; k++ ) //for #2
        {
            const string& filename = imagelist[i*2+k];
            pathfile.append(path).append(filename);
            Mat img = imread(pathfile, 0);
            pathfile="";
            if(img.empty())
                break;
            if( imageSize == Size() )
                imageSize = img.size();
            else if( img.size() != imageSize ) //mensaje de error
            {
                info="The image ";
                archivo=QString::fromStdString(filename);
                info.append(archivo).append(" has the size different from the first image size. Skipping the pair\n");
                break;
            }
            bool found = false;
            vector<Point2f>& corners = imagePoints[k][j];
            for( int scale = 1; scale <= maxScale; scale++ ) //for #3
            {
                Mat timg;
                if( scale == 1 )
                    timg = img;
                else
                    resize(img, timg, Size(), scale, scale);
                found = findChessboardCorners(timg, boardSize, corners,
                    CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
                if( found )
                {
                    if( scale > 1 )
                    {
                        Mat cornersMat(corners);
                        cornersMat *= 1./scale;
                    }
                    break;
                }
            } //end for #3
            if( displayCorners ) //muestra la esquinas encontradas en el tablero
            {

                archivo=QString::fromStdString(filename);
                info.append(archivo);
                //PrintInfo(error);
                //cout << filename << endl;
                Mat cimg, cimg1;
                cvtColor(img, cimg, COLOR_GRAY2BGR);
                drawChessboardCorners(cimg, boardSize, corners, found);
                double sf = 640./MAX(img.rows, img.cols);
                resize(cimg, cimg1, Size(), sf, sf);
                imshow("corners", cimg1);
                char c = (char)waitKey(500);
                if( c == 27 || c == 'q' || c == 'Q' ) //Allow ESC to quit
                    exit(-1);
            }
            else
                putchar('.');
            if( !found )
                break;
            cornerSubPix(img, corners, Size(11,11), Size(-1,-1),
                         TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
                                      30, 0.01));
        }//end for #2
        if( k == 2 )
        {
            goodImageList.push_back(imagelist[i*2]);
            goodImageList.push_back(imagelist[i*2+1]);
            j++;
        }
    } //end for #1

    number=QString::number(j);
    info.append(number).append(" pairs have been successfully detected.\n");
    //PrintInfo(error);
    //cout << j << " pairs have been successfully detected.\n";
    nimages = j;
    if( nimages < 2 ) //informa error: muy pocos pares para calibrar
    {
        error="Error: too little pairs to run the calibration\n";
        PrintInfo(error);
        return error;
    }

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    objectPoints.resize(nimages);

    for( i = 0; i < nimages; i++ ) //for #1 recorre todas las imagenes
    {
        for( j = 0; j < boardSize.height; j++ )// for #2 recorre cada columna del tablero
            for( k = 0; k < boardSize.width; k++ ) //for #3 recorre cada fila del tablero
                objectPoints[i].push_back(Point3f(j*squareSize, k*squareSize, 0));
    }

    info.append("Running stereo calibration ...\n");

    Mat cameraMatrix[2], distCoeffs[2];
    cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
    cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
    Mat R, T, E, F;

    double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                    cameraMatrix[0], distCoeffs[0],
                    cameraMatrix[1], distCoeffs[1],
                    imageSize, R, T, E, F,
                    TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                    CV_CALIB_FIX_ASPECT_RATIO +
                    CV_CALIB_ZERO_TANGENT_DIST +
                    CV_CALIB_SAME_FOCAL_LENGTH +
                    CV_CALIB_RATIONAL_MODEL +
                    CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);

    number=QString::number(rms);
    info.append("done with RMS error=").append(number).append("\n");

// CALIBRATION QUALITY CHECK
// because the output fundamental matrix implicitly
// includes all the output information,
// we can check the quality of calibration using the
// epipolar geometry constraint: m2^t*F*m1=0
    double err = 0;
    int npoints = 0;
    vector<Vec3f> lines[2];
    for( i = 0; i < nimages; i++ )
    {
        int npt = (int)imagePoints[0][i].size();
        Mat imgpt[2];
        for( k = 0; k < 2; k++ )
        {
            imgpt[k] = Mat(imagePoints[k][i]);
            undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
            computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
        }
        for( j = 0; j < npt; j++ )
        {
            double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
                                imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
                           fabs(imagePoints[1][i][j].x*lines[0][j][0] +
                                imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }
    number=QString::number(err/npoints) ;
    info.append("average reprojection err = ").append(number).append("\n");

    // save intrinsic parameters
    pathfile="";
    pathfile.append(path).append("intrinsics.yml");
    //
    FileStorage fs(pathfile, CV_STORAGE_WRITE);
    if( fs.isOpened() )
    {
        fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
            "M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
        fs.release();
    }
    else{
        error= "Error: can not save the intrinsic parameters\n";
        PrintInfo(error);
    }

    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];

    //rectify the image
    stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, R, T, R1, R2, P1, P2, Q,
                  CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);

    pathfile="";
    pathfile.append(path).append("extrinsics.yml");
    fs.open(pathfile, CV_STORAGE_WRITE);
    pathfile="";
    if( fs.isOpened() )
    {
        fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
        fs.release();
    }
    else{
        error="Error: can not save the extrinsics parameters\n";
        PrintInfo(error);
    }

    // OpenCV can handle left-right
    // or up-down camera arrangements
    bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

// COMPUTE AND DISPLAY RECTIFICATION
    if( !showRectified )
        return "";

    Mat rmap[2][2];
// IF BY CALIBRATED (BOUGUET'S METHOD)
    if( useCalibrated )
    {
        // we already computed everything
    }
// OR ELSE HARTLEY'S METHOD
    else
 // use intrinsic parameters of each camera, but
 // compute the rectification transformation directly
 // from the fundamental matrix
    {
        vector<Point2f> allimgpt[2];
        for( k = 0; k < 2; k++ )
        {
            for( i = 0; i < nimages; i++ )
                std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
        }
        F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
        Mat H1, H2;
        stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

        R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
        R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
        P1 = cameraMatrix[0];
        P2 = cameraMatrix[1];
    }

    //Precompute maps for cv::remap()
    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

    Mat canvas;
    double sf;
    int w, h;
    if( !isVerticalStereo )
    {
        sf = 600./MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h, w*2, CV_8UC3);
    }
    else
    {
        sf = 300./MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h*2, w, CV_8UC3);
    }

    for( i = 0; i < nimages; i++ )
    {
        for( k = 0; k < 2; k++ )
        {
            const string& filename = goodImageList[i*2+k];
            pathfile.append(path).append(filename);
            //Mat img = imread(pathfile, 0);

            Mat img = imread(pathfile, 0), rimg, cimg;
            pathfile="";
            remap(img, rimg, rmap[k][0], rmap[k][1], CV_INTER_LINEAR);
            cvtColor(rimg, cimg, COLOR_GRAY2BGR);
            Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h));
            resize(cimg, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
            if( useCalibrated )
            {
                Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
                          cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
                rectangle(canvasPart, vroi, Scalar(0,0,255), 3, 8);
            }
        }

        if( !isVerticalStereo )
            for( j = 0; j < canvas.rows; j += 16 )
                line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
        else
            for( j = 0; j < canvas.cols; j += 16 )
                line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
        imshow("rectified", canvas);
        char c = (char)waitKey();
        if( c == 27 || c == 'q' || c == 'Q' )
            break;
    }
    return info;
}

void MainWindow::on_actionCalibrar_camara_triggered()
{




    Size boardSize;
    string imagelistfn;
    bool showRectified = true;

    QMessageBox msgBox;
    QString msg,msg2;

    imagelistfn = "calibrate/stereo_calib.xml";
    boardSize = Size(9, 6); //9,6
    vector<string> imagelist;

    ui->plainTextEdit->appendPlainText("Leyendo lista de imagenes \n");

    //this->("Leyendo lista de imagenes");

    //msuarez lectura de imagenes desde xml. Se puede mejorar esta funcion tomando captura de imagen desde camara web.
    bool ok = readStringList(imagelistfn, imagelist);
    if(!ok || imagelist.empty())
    {
        //cout << "can not open " << imagelistfn << " or the string list is empty" << endl;

        msg2=QString::fromStdString(imagelistfn);
        msg.append("can not open ").append(msg2).append(" or the string list is empty");
        msgBox.setText(msg);
        msgBox.exec();
        //return print_help();
    }
qDebug("imagelistfn=%s",imagelistfn.data());
/*QDir dir;
QString currentDirectory = dir.absolutePath();
string currentpath=currentDirectory.toStdString();
qDebug("\n imagelistfn=%s", currentpath.data());*/
    msg=StereoCalib(imagelist, boardSize, true, showRectified);
    ui->plainTextEdit->appendPlainText(msg);

}
