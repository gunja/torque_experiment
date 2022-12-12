#include "joint_setting.h"
//#include "ui_joint_setting.h"


#include <iostream>

JointSettings::JointSettings(QWidget * parent, Qt::WindowFlags f):
    QWidget(parent, f), initialValueIn(0)
    , ecnWrapper(nullptr)
    , motor_link_reduction(0.0)
{

    ui.setupUi(this);
    ui.a_spin_motor_index->setValue( initialValueIn);
    ui.a_index_display->setText( QString("%1").arg(initialValueIn));
}

JointSettings::~JointSettings()
{
}

void JointSettings::on_a_write_XCels_clicked()
{
    auto motorIdx = ui.a_index_display->text().toInt();
    auto newAccel = ui.a_accel_edit->text().toInt();
    auto newDecel = ui.a_decel_edit->text().toInt();

    if (ecnWrapper == nullptr)
        return;

    ecnWrapper->setAcceleration(motorIdx, newAccel * 600);
    ecnWrapper->setDeceleration(motorIdx, newDecel * 600);

    fillConfigs( ui.a_spin_motor_index->value());
}

void JointSettings::on_a_spin_motor_index_valueChanged(int nv)
{
    fillConfigs(nv);
}

void JointSettings::fillConfigs()
{
    auto val = ui.a_index_display->text().toInt();
    fillConfigs(val);
}

void JointSettings::fillConfigs(int nv)
{
    ui.a_index_display->setText( QString("%1").arg(nv));
    if (ecnWrapper != nullptr) {
        uint32_t accel;
        ecnWrapper->fillUINT32(nv, 0x6083, 0, accel);
        uint32_t decel;
        ecnWrapper->fillUINT32(nv, 0x6084, 0, decel);
        updateDisplayedAccelDecel(accel, decel);
    }
}

void JointSettings::on_a_disable_motor_clicked()
{
    auto motorIdx = ui.a_index_display->text().toInt();
    if (ecnWrapper == nullptr)
        return;

    ecnWrapper->disableMotor(motorIdx);
}

void JointSettings::on_a_start_rotation_clicked()
{
    auto motorIdx = ui.a_index_display->text().toInt();
    auto v = ui.a_speed->text().toDouble();
    auto dura = ui.a_rotate_duration->text().toDouble();
    if (ecnWrapper == nullptr)
        return;

    ecnWrapper->startDurableRotation(motorIdx, v, dura);
}

void JointSettings::on_a_stop_rotation_clicked()
{
    auto motorIdx = ui.a_index_display->text().toInt();
    if (ecnWrapper == nullptr)
        return;

    ecnWrapper->startDurableRotation(motorIdx, 0.0, 0.0);
}

void JointSettings::on_a_clear_error_clicked()
{
    auto motorIdx = ui.a_index_display->text().toInt();
    if (ecnWrapper == nullptr)
        return;

    ecnWrapper->clearError(motorIdx);
}

void JointSettings::on_a_speed_valueChanged(int nv)
{
    ui.a_visible->setText( QString("%1").arg(nv * motor_link_reduction ));
}

void JointSettings::updateIndexAndSpin(int dVal)
{
    ui.a_spin_motor_index->setValue(dVal);
    ui.a_index_display->setText( QString("%1").arg(dVal));
}

int JointSettings::getDisplayedIndex() const
{
    return ui.a_index_display->text().toInt();
}

void JointSettings::updateDisplayedAccelDecel(int accel, int decel)
{
    ui.a_accel_read->setText( QString("%1").arg( 1. / 600. * accel));
    ui.a_decel_read->setText( QString("%1").arg( 1. / 600. * decel));
}

void JointSettings::displayRemaining(double delta)
{
    ui.a_rem_rotation->setText( QString("%1").arg( delta));
}

void JointSettings::on_a_position_go_clicked()
{
    auto motorIdx = ui.a_index_display->text().toInt();
    if (ecnWrapper == nullptr)
        return;
    ecnWrapper->startGoToPosition(motorIdx,
        ui.a_position->value(), ui.a_position_velocity->value());
}

void JointSettings::rotateButtoned(bool st, int sign)
{
    auto motorIdx = ui.a_index_display->text().toInt();
    if (ecnWrapper == nullptr)
        return;
    if (st) {
        auto v = abs(ui.a_speed->text().toDouble());
        ecnWrapper->startDurableRotation(motorIdx, sign *v, 1000.);
    } else {
        ecnWrapper->startDurableRotation(motorIdx, 0.0, 0.0);
    }
}

void JointSettings::on_a_rotate_negative_pressed()
{
    std::cout<<"positive. st=true"<<std::endl;
    rotateButtoned( true, -1);
}

void JointSettings::on_a_rotate_negative_released()
{
    std::cout<<"positive. st=false"<<std::endl;
    rotateButtoned( false, -1);
}

void JointSettings::on_a_rotate_positive_pressed()
{
    std::cout<<"positive. st=true"<<std::endl;
    rotateButtoned( true, 1);
}

void JointSettings::on_a_rotate_positive_released()
{
    std::cout<<"positive. st=false"<<std::endl;
    rotateButtoned( false, 1);
}

