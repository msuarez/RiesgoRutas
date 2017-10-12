#ifndef CALIBRATECAM_H
#define CALIBRATECAM_H

#include <QMainWindow>

namespace Ui {
class calibrateCam;
}

class calibrateCam : public QMainWindow
{
    Q_OBJECT

public:
    explicit calibrateCam(QWidget *parent = 0);
    ~calibrateCam();

private slots:
    void on_btnImageCapture_clicked();

    int on_btnCalibrate_clicked();

    void on_btnfileimage_clicked();

    void on_rbtnviseocapture_clicked();

private:
    Ui::calibrateCam *ui;
};

#endif // CALIBRATECAM_H
