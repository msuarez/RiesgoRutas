#ifndef CALIBRATESETTING_H
#define CALIBRATESETTING_H

#include <QWidget>
#include"mainwindow.h"

namespace Ui {
class calibratesetting;
}

class calibratesetting : public QWidget
{
    Q_OBJECT

public:
    explicit calibratesetting(QWidget *parent = 0);
    ~calibratesetting();

private slots:
    void on_calibratebtn_clicked();

private:
    Ui::calibratesetting *ui;
    //void show(MainWindow *principal);
};

#endif // CALIBRATESETTING_H
