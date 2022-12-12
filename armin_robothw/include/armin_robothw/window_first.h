#pragma once

#include <sys/time.h>

#include <iostream>
#include <list>
#include <memory>
#include <string>
#include <mutex>

#include <QDialog>
#include <QTimer>
#include <QLineEdit>

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include "ui_window_first.h"
#include "ecn.h"
#include "festo_over_ec.h"
#include "ekxx.h"

#include  "ecn_wrapper.h"
#include "moment_instruction.h"

#define ZERO_STREAM_FILENAME "zero_positions.zrs"
#define JOINTS_COUNT 2

class WindowFirst : public QDialog, public ECNWrapper
{
  Q_OBJECT

    public:
        WindowFirst(QWidget *parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags());
        ~WindowFirst();
        void fillUINT32( const int slave, const int slot, const int subslot, uint32_t &dst) override;

public Q_SLOTS:
        void on_buttonBox_accepted() { std::cout<<"accepted"<<std::endl;};
        void on_buttonBox_rejected() { std::cout<<"rejected"<<std::endl;};
        void on_ec_start_clicked();
        void on_ec_operational_clicked();

        void on_updateCyclicTimer_timeout();

        void on_all_disable_button_clicked();
        void on_home_all_button_clicked();

        void on_open_file_button_clicked();
        void on_start_scenario_button_clicked();
        void on_reset_list_button_clicked();

        void on_clear_all_button_clicked();
        void on_go_initial_button_clicked();
        void on_all_zero_clicked();

        void on_write_zero_button_clicked();

        void on_open_grasp_button_pressed();
        void on_open_grasp_button_released();
        void on_close_grasp_button_pressed();
        void on_close_grasp_button_released();

        void startDurableRotation(int mtr, double v, double d) override;
        void disableMotor(int) override;
        void clearError(int mtr) override;
        void setAcceleration(int mtr, int value) override;
        void setDeceleration(int mtr, int value) override;
        virtual void startGoToPosition(int mtr, int position, int velocity) override;
    private:
        Ui::Dialog ui;
        QLineEdit * modesLineEdits;
        QLineEdit * statusLineEdits;
        QLineEdit * positionLineEdits;
        QLineEdit * errorLineEdits;
        QLineEdit * velocityLineEdits;
        QLineEdit * currentLineEdits;
        QLineEdit * warningLineEdits;
        QLineEdit * digiInputLineEdits;
        EtherCATNetwork ecn;
        void fillConfigurationElements();
        void fillConfigA6();
        void fillConfigA5();
        void fillConfigA4();
        void fillConfigA3();
        void fillConfigA2();
        void fillConfigA1();
        FestoController * f_cnt;
        EKXX * ek1828;
        void reflectControlsToOneAnother();
        bool MotorIndexOperationAllowed(int) const override;
        QTimer updateCyclicTimer;
        struct timeval *StartTimers;

        const double FestoSmallReductorMulti = 1./3.;
        const double Cilindric_A6_transmission = 56. / 35.;
        const double Wave_A6_transmission = 1. / 121.;
        const double A6_motor_link_reduction = FestoSmallReductorMulti / Cilindric_A6_transmission * Wave_A6_transmission;

        const double Cilindric_A5_transmission = 56. / 35.;
        const double Wave_A5_transmission = 1. / 161.;
        const double A5_motor_link_reduction =FestoSmallReductorMulti / Cilindric_A5_transmission * Wave_A5_transmission;

        
        const double Cilindric_A4_transmission = 56. / 35.;
        const double Wave_A4_transmission = 1. / 161.;
        const double A4_motor_link_reduction =FestoSmallReductorMulti / Cilindric_A4_transmission * Wave_A4_transmission;

        const double Cilindric_A3_transmission = 74. / 35.;
        const double Wave_A3_transmission = 1. / 161.;
        const double A3_motor_link_reduction =FestoSmallReductorMulti / Cilindric_A3_transmission * Wave_A3_transmission;

        const double Cilindric_A2_transmission = 74. / 35.;
        const double Wave_A2_transmission = 1. / 161.;
        const double A2_motor_link_reduction =FestoSmallReductorMulti / Cilindric_A2_transmission * Wave_A2_transmission;

        const double Cilindric_A1_transmission = 74. / 35.;
        const double Wave_A1_transmission = 1. / 161.;
        const double A1_motor_link_reduction =FestoSmallReductorMulti / Cilindric_A1_transmission * Wave_A1_transmission;


        const double A5_A6_correction = A5_motor_link_reduction / FestoSmallReductorMulti * Cilindric_A6_transmission;

        const double A4_A5_correction = A4_motor_link_reduction / FestoSmallReductorMulti * Cilindric_A5_transmission;
        const double A4_A6_correction = A4_motor_link_reduction / FestoSmallReductorMulti * Cilindric_A6_transmission;

        const double A3_A4_correction = A3_motor_link_reduction / FestoSmallReductorMulti * Cilindric_A4_transmission;
        const double A3_A5_correction = A3_motor_link_reduction / FestoSmallReductorMulti * Cilindric_A5_transmission;
        const double A3_A6_correction = A3_motor_link_reduction / FestoSmallReductorMulti * Cilindric_A6_transmission;

        const double A2_A3_correction = A2_motor_link_reduction / FestoSmallReductorMulti * Cilindric_A3_transmission;
        const double A2_A4_correction = A2_motor_link_reduction / FestoSmallReductorMulti * Cilindric_A4_transmission;
        const double A2_A5_correction = A2_motor_link_reduction / FestoSmallReductorMulti * Cilindric_A5_transmission;
        const double A2_A6_correction = A2_motor_link_reduction / FestoSmallReductorMulti * Cilindric_A6_transmission;

    
        const double A1_A2_correction = A1_motor_link_reduction / FestoSmallReductorMulti * Cilindric_A2_transmission;
        const double A1_A3_correction = A1_motor_link_reduction / FestoSmallReductorMulti * Cilindric_A3_transmission;
        const double A1_A4_correction = A1_motor_link_reduction / FestoSmallReductorMulti * Cilindric_A4_transmission;
        const double A1_A5_correction = A1_motor_link_reduction / FestoSmallReductorMulti * Cilindric_A5_transmission;
        const double A1_A6_correction = A1_motor_link_reduction / FestoSmallReductorMulti * Cilindric_A6_transmission;


        std::list< std::shared_ptr<MomentInstructions> > instructionsList;
        struct timeval clickScenario;
        std::list< std::shared_ptr<MomentInstructions> >::iterator currentStep;
        std::recursive_mutex instructionsMutex;

        void tickNextScenarioStep();
        void createLayoutWidgets();
        void destroyLayoutWidgets();
        void reflectCyclicDataToControls();

        int knownZeroes[JOINTS_COUNT];
        void loadZeroPositions();
        bool initialHomingDone;

        ros::NodeHandle n;
        ros::Publisher pub;
        std::vector<std::string> jointNames;

};

