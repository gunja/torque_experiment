#pragma once

#include <QWidget>
#include "ui_joint_setting.h"

#include "ecn_wrapper.h"


class JointSettings : public QWidget
{
    Q_OBJECT

    public:
        JointSettings(QWidget * parent = 0, Qt::WindowFlags f = 0);
        ~JointSettings();
        int getDisplayedIndex() const;
        void updateDisplayedAccelDecel(int, int);
        void setECNWrapper(ECNWrapper *ptr) { ecnWrapper = ptr;};
        void setMotorLinkReduction( double _mlr) { motor_link_reduction = _mlr;};
        void fillConfigs();
        void displayRemaining(double);

  public slots:
    void on_a_write_XCels_clicked();
    void on_a_spin_motor_index_valueChanged(int);
    void on_a_disable_motor_clicked();
    void on_a_start_rotation_clicked();
    void on_a_stop_rotation_clicked();
    void on_a_clear_error_clicked();
    void on_a_speed_valueChanged(int);
    void updateIndexAndSpin(int);
    void on_a_position_go_clicked();
    void on_a_rotate_negative_pressed();
    void on_a_rotate_negative_released();
    void on_a_rotate_positive_pressed();
    void on_a_rotate_positive_released();

  private:
    Ui::Form ui;
    int initialValueIn;
    void fillConfigs(int nv);
    ECNWrapper *ecnWrapper;
    double motor_link_reduction;
    void rotateButtoned(bool st, int sign);
};

