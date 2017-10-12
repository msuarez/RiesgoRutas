#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void appendPlainText(QString text);

private slots:
    void on_actionCargar_Imagen_1_triggered();

    void on_actionCargar_Imagen_2_triggered();

    void on_actionSalir_triggered();

    void on_actionCalibrar_camara_triggered();

    void on_actionStereo_Calibration_triggered();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
