#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <vector>

#include <QString>
#include <QFileDialog>

#ifndef Q_MOC_RUN
#include <sensor_msgs/JointState.h>
#include <std_msgs/Header.h>
#include <ros/time.h>
#endif

#include "window_first.h"
#include "ui_window_first.h"

WindowFirst::WindowFirst(QWidget *parent, Qt::WindowFlags f):
    QDialog( parent, f)
    , f_cnt(nullptr)
    , ek1828(nullptr)
    , updateCyclicTimer(this)
    , StartTimers(nullptr)
    , initialHomingDone(false)
{
    ui.setupUi(this);
    ui.tab_6->updateIndexAndSpin(6);
    ui.tab_6->setMotorLinkReduction(A6_motor_link_reduction);
    ui.tab_6->setECNWrapper(this);
    
    ui.tab_5->updateIndexAndSpin(5);
    ui.tab_5->setMotorLinkReduction(A5_motor_link_reduction);
    ui.tab_5->setECNWrapper(this);

    ui.tab_4->updateIndexAndSpin(4);
    ui.tab_4->setMotorLinkReduction(A4_motor_link_reduction);
    ui.tab_4->setECNWrapper(this);

    ui.tab_3->updateIndexAndSpin(3);
    ui.tab_3->setMotorLinkReduction(A3_motor_link_reduction);
    ui.tab_3->setECNWrapper(this);

    ui.tab_2->updateIndexAndSpin(2);
    ui.tab_2->setMotorLinkReduction(A2_motor_link_reduction);
    ui.tab_2->setECNWrapper(this);

    ui.tab->updateIndexAndSpin(1);
    ui.tab->setMotorLinkReduction(A1_motor_link_reduction);
    ui.tab->setECNWrapper(this);

    createLayoutWidgets();

    currentStep=instructionsList.end();

    updateCyclicTimer.setInterval(5);
    connect( &updateCyclicTimer, SIGNAL(timeout()),
        this, SLOT(on_updateCyclicTimer_timeout()));

    loadZeroPositions();

    pub = n.advertise<sensor_msgs::JointState>("/joint_states", 2);
    jointNames = std::move( std::vector<std::string>( { "a1_joint", "a2_joint", "a3_joint", "a4_joint", "a5_joint", "a6_joint"}) );
};

WindowFirst::~WindowFirst() {
    if (f_cnt != nullptr) {
        std::ofstream zeroFile(ZERO_STREAM_FILENAME);
        if (zeroFile.is_open()) {
            for(auto i=0; i < ecn.getDiscoveredSlavesCount() && i < JOINTS_COUNT; ++i)
            {
                zeroFile<<(knownZeroes[i] + f_cnt[i].getPositionValue())<<std::endl;
            }
            zeroFile.close();
        } else {
        // TODO notify on the problem with file opening
        }
    }
    if (f_cnt != nullptr)
    {
        auto i = ecn.getDiscoveredSlavesCount();
        for(; i > 0; --i) {
            f_cnt[i-1].~FestoController();
        }
        std::free(f_cnt);
        f_cnt = nullptr;
    }
    if (StartTimers != nullptr) {
        std::free(StartTimers);
        StartTimers = nullptr;
    }
    destroyLayoutWidgets();
}

void WindowFirst::on_ec_start_clicked()
{
    QString  iface_name = ui.ec_ifname->text();
    auto rv = ecn.startNetwork( iface_name.toLocal8Bit().constData());
    if (rv) {
        ui.ec_slavecount->setText(QString("%1").arg(static_cast<int>( ecn.getECKnownSlavesCount())));
        ui.ec_state->setText(QString("%1").arg(ecn.getNetState()));
        fillConfigurationElements();
    } else {
        ui.ec_slavecount->setText("Failed to find any slaves");
    }
}


void WindowFirst::on_ec_operational_clicked()
{
    auto r = ecn.SwitchSafeOperational();
    if (r) {
        ui.ec_state->setText(QString("%1").arg(ecn.getNetState()));
        if (ecn.isSafeOperational() && f_cnt == nullptr ) {
            f_cnt = reinterpret_cast<FestoController*>(std::calloc(
                        ecn.getECKnownSlavesCount(), sizeof(FestoController)));
            StartTimers = reinterpret_cast<struct timeval*>(std::calloc( ecn.getECKnownSlavesCount(), sizeof(struct timeval)));
            for(int i = 0; i < ecn.getECKnownSlavesCount(); ++i) {
                new (&f_cnt[i]) FestoController(
                    *(reinterpret_cast<Master2Slave*>(ecn.getOutputPointer(i+1))),
                    *(reinterpret_cast<Slave2Master*>(ecn.getInputPointer(i+1))),
                    ecn, i + 1);
                StartTimers[i].tv_sec = 0; StartTimers[i].tv_usec = 0;
            }
            // ATT hard bounding. noone knows how it will really work
            reflectControlsToOneAnother();

            for(auto ec_i = 1; ec_i <= ecn.getDiscoveredSlavesCount(); ++ec_i)
            {
                //if (std::string("EK1828") == ecn.getSlaveName(ec_i))
                if( strcmp("EK1828", ecn.getSlaveName(ec_i)) == 0 )
                if(ek1828==nullptr)
                    ek1828 = new EKXX(
                      *(reinterpret_cast<Ekxx_Region_m2s*> (ecn.getOutputPointer(ec_i))),
                      *(reinterpret_cast<Ekxx_Region_s2m*> (ecn.getInputPointer(ec_i))),
                            ecn, ec_i);
            }
        }
        r = ecn.SwitchOperational();
        if (r) {
            ui.ec_state->setText(QString("%1").arg(ecn.getNetState()));
            updateCyclicTimer.start();
        }
    } else {
    }
}

void WindowFirst::fillConfigurationElements()
{
    // TODO
    fillConfigA6();
    fillConfigA5();
    fillConfigA4();
    fillConfigA3();
    fillConfigA2();
    fillConfigA1();
}

void WindowFirst::fillConfigA6()
{
    ui.tab_6->fillConfigs();
}

void WindowFirst::fillConfigA5()
{
    ui.tab_5->fillConfigs();
  std::cout<<"WindowFirst::fillConfigA5 called"<<std::endl;
}

void WindowFirst::fillConfigA4()
{
    ui.tab_4->fillConfigs();
  std::cout<<"WindowFirst::fillConfigA4 called"<<std::endl;
}

void WindowFirst::fillConfigA3()
{
    ui.tab_3->fillConfigs();
  std::cout<<"WindowFirst::fillConfigA3 called"<<std::endl;
}

void WindowFirst::fillConfigA2()
{
    ui.tab_2->fillConfigs();
  std::cout<<"WindowFirst::fillConfigA2 called"<<std::endl;
}

void WindowFirst::fillConfigA1()
{
    ui.tab->fillConfigs();
  std::cout<<"WindowFirst::fillConfigA1 called"<<std::endl;
}

void WindowFirst::fillUINT32( const int slave, const int slot, const int subslot, uint32_t &dst)
{
    dst = ecn.getSlotDataUINT32(slave, slot, subslot);
}

bool WindowFirst::MotorIndexOperationAllowed(int idx) const
{
    if (f_cnt == nullptr) return false;
    if ( idx < 1 || idx > ecn.getDiscoveredSlavesCount())
        return false;

    return true;
}

bool operator<(const struct timeval &tvL, const struct timeval &tvR)
{
    if( tvL.tv_sec < tvR.tv_sec ) return true;
    if( tvL.tv_sec > tvR.tv_sec ) return false;
    if( tvL.tv_usec < tvR.tv_usec) return true;
    return false;
}

void WindowFirst::on_updateCyclicTimer_timeout()
{
    if (f_cnt == nullptr) return;
    struct timeval tv;
    gettimeofday( &tv, NULL);
    if (StartTimers != nullptr) {
        for(auto i=0; i < ecn.getECKnownSlavesCount(); ++i) {
            if( StartTimers[i] < tv) {
               if (StartTimers[i].tv_sec != 0)
                f_cnt[i].setTargetVelocity(0);
            } else {
                if (i==5) {
                    double delta = StartTimers[i].tv_sec - tv.tv_sec + 1e-6 * (StartTimers[i].tv_usec - tv.tv_usec);
                    ui.tab_6->displayRemaining(delta);
                }
                if (i==4) {
                    double delta = StartTimers[i].tv_sec - tv.tv_sec + 1e-6 * (StartTimers[i].tv_usec - tv.tv_usec);
                    ui.tab_5->displayRemaining(delta);
                }
                if (i==3) {
                    double delta = StartTimers[i].tv_sec - tv.tv_sec + 1e-6 * (StartTimers[i].tv_usec - tv.tv_usec);
                    ui.tab_4->displayRemaining(delta);
                }
                if (i==2) {
                    double delta = StartTimers[i].tv_sec - tv.tv_sec + 1e-6 * (StartTimers[i].tv_usec - tv.tv_usec);
                    ui.tab_3->displayRemaining(delta);
                }
                if (i==1) {
                    double delta = StartTimers[i].tv_sec - tv.tv_sec + 1e-6 * (StartTimers[i].tv_usec - tv.tv_usec);
                    ui.tab_2->displayRemaining(delta);
                }
                if (i==0) {
                    double delta = StartTimers[i].tv_sec - tv.tv_sec + 1e-6 * (StartTimers[i].tv_usec - tv.tv_usec);
                    ui.tab->displayRemaining(delta);
                }
            }
        }
    }
  {
    std::lock_guard<std::mutex> lock {ecn.getGuard()};
    for(auto i = 0; i < 2; ++i) {
            f_cnt[i].updateOutputs();
    }
    tickNextScenarioStep();
  }
    reflectCyclicDataToControls();

    sensor_msgs::JointState msg;
    msg.name = jointNames;
    msg.header = std_msgs::Header();
    msg.header.stamp = ros::Time::now();
    int32_t knownPositions[JOINTS_COUNT];
    for(auto i = 0; i < ecn.getECKnownSlavesCount() && i < JOINTS_COUNT; ++i) {
        knownPositions[i] = f_cnt[i].getPositionValue();
        knownPositions[i] += knownZeroes[i];
    }
    //improper initialization of variable do not allow to cycle following
    // calculation
    double val = A1_motor_link_reduction * knownPositions[0];
    const double popugai_to_rads = 1. /1800. * M_PI;
    msg.position.push_back(popugai_to_rads * val);

    val = A2_motor_link_reduction * knownPositions[1];
#ifdef ENABLE_CORRECTIONS
    val += knownPositions[0] * A1_A2_correction;
#endif
    msg.position.push_back(popugai_to_rads * val);

    val = A3_motor_link_reduction * knownPositions[2];
#ifdef ENABLE_CORRECTIONS
    val += knownPositions[0] * A1_A3_correction;
    val += knownPositions[1] * A2_A3_correction;
#endif
    msg.position.push_back(popugai_to_rads * val);
    val = A4_motor_link_reduction * knownPositions[3];
#ifdef ENABLE_CORRECTIONS
    val -= knownPositions[0] * A1_A4_correction;
    val -= knownPositions[1] * A2_A4_correction;
    val -= knownPositions[2] * A3_A4_correction;
#endif
    msg.position.push_back(popugai_to_rads * val);
    val = - A5_motor_link_reduction * knownPositions[4];
#ifdef ENABLE_CORRECTIONS
    val += knownPositions[0] * A1_A5_correction;
    val += knownPositions[1] * A2_A5_correction;
    val += knownPositions[2] * A3_A5_correction;
    val += knownPositions[3] * A4_A5_correction;
#endif
    msg.position.push_back(popugai_to_rads * val);
    val = A6_motor_link_reduction * knownPositions[5];
#ifdef ENABLE_CORRECTIONS
    val += knownPositions[0] * A1_A6_correction;
    val += knownPositions[1] * A2_A6_correction;
    val += knownPositions[2] * A3_A6_correction;
    val += knownPositions[3] * A4_A6_correction;
    val += knownPositions[4] * A5_A6_correction;
#endif
    msg.position.push_back(popugai_to_rads * val);

    //pub.publish(msg);
}

void WindowFirst::reflectControlsToOneAnother()
{
    f_cnt[1].addCorrector(f_cnt + 0, A1_A2_correction);
}

void WindowFirst::disableMotor(int mtrNum)
{
    if (!MotorIndexOperationAllowed(mtrNum)) return;
    f_cnt[mtrNum -1].disableMotor();
    StartTimers[mtrNum -1].tv_sec =0;
}

struct timeval operator+(const struct timeval &a, double v)
{
    struct timeval o(a);
    double dump;
    o.tv_sec += static_cast<int>(v);
    o.tv_usec += 1000000 * std::modf(v, &dump);
    if ( o.tv_usec > 1000000) {
        o.tv_usec -= 1000000;
        o.tv_sec++;
    }
    return std::move(o);
}

struct timeval & operator+=( struct timeval &l, double v)
{
    l = l + v;
    return l;
}

std::ostream & operator<<(std::ostream &st, const struct timeval &t)
{
    auto w = st.width();
    st<<t.tv_sec<<"."<<std::setw(6)<<t.tv_usec<<std::setw(w);
    return st;
}

void WindowFirst::startDurableRotation(int mtr, double v, double d)
{
std::cout<<"startDurableRotation called for mtr="<<mtr<<"    with vitess "<<v<<"   and duration "<<d<<std::endl;
    if (!MotorIndexOperationAllowed(mtr)) return;
    f_cnt[mtr - 1].setTargetVelocity( static_cast<int>( 60 * v));
    f_cnt[mtr - 1].startEnabling();
    //double dumm;
    // TODO start back counter
    gettimeofday( StartTimers + mtr - 1, NULL);
std::cout<<"timer  was: "<<StartTimers[mtr - 1]<<std::endl;
    StartTimers[mtr - 1] += d;
std::cout<<"timer now: "<<StartTimers[mtr - 1]<<std::endl;
/*
    StartTimers[mtr - 1].tv_sec += static_cast<int>(d);
    StartTimers[mtr - 1].tv_usec += 1000000 * std::modf(d, &dumm);
    if ( StartTimers[mtr - 1].tv_usec > 1000000) {
        StartTimers[mtr - 1].tv_usec -= 1000000;
        StartTimers[mtr - 1].tv_sec++;
    }
*/
}

void WindowFirst::clearError(int mtr)
{
    if (!MotorIndexOperationAllowed(mtr)) return;
    f_cnt[mtr - 1].clearError();
}

void WindowFirst::setAcceleration(int mtr, int value)
{
    ecn.setSlotDataUINT32( mtr, 0x6083, 0, value);
}

void WindowFirst::setDeceleration(int mtr, int value)
{
    ecn.setSlotDataUINT32( mtr, 0x6084, 0, value);
}

void WindowFirst::on_all_disable_button_clicked()
{
    if (f_cnt == nullptr) return;

    for(auto i=0; i < ecn.getDiscoveredSlavesCount(); ++i) {
        f_cnt[i].disableMotor();
        StartTimers[i].tv_sec =0;
    }
}

void WindowFirst::on_home_all_button_clicked()
{
    if (f_cnt == nullptr) return;

    if (initialHomingDone) {
        for(auto i=0; i < ecn.getDiscoveredSlavesCount() && i < JOINTS_COUNT; ++i) {
            knownZeroes[i] += f_cnt[i].getPositionValue();
        }
    }

    for (auto i=0; i < ecn.getDiscoveredSlavesCount(); ++i)
    {
        f_cnt[i].completeHoming();
    }
    initialHomingDone = true;
}

void WindowFirst::on_open_file_button_clicked()
{
  {std::lock_guard<std::recursive_mutex> lock{ instructionsMutex};
    instructionsList.clear();
    currentStep = instructionsList.end();
  }
    std::cout<<"open file button"<<std::endl;
    auto fileName = QFileDialog::getOpenFileName(this,
    tr("Open Scenario"), ".", tr("Scenario files (*.scN)"));
    if (fileName == "") {
        ui.scenario_runtime->setText("0.0");
        ui.step_num->setText("0");
        ui.max_step_num->setText("0");
        return;
    }
    std::cout<<"Selected "<<fileName.toLocal8Bit().constData()<<std::endl;
    std::ifstream is(fileName.toLocal8Bit().constData());
    char lineBuffer[2000];
    while( ! is.eof()) {
        is.getline(lineBuffer, 2000);
        if (is.fail()) continue;
        auto nn = std::shared_ptr<MomentInstructions> (MomentInstructions::fromLine(lineBuffer));
        if (nn != nullptr)
            instructionsList.push_back(nn);
    }
    is.close();
    ui.step_num->setText("0 done");
    ui.max_step_num->setText(QString("%1 loaded").arg( instructionsList.size()));
    instructionsList.sort( [](const std::shared_ptr<MomentInstructions> &a, const std::shared_ptr<MomentInstructions> &b) { return (*a) < (*b); });
}

void WindowFirst::on_start_scenario_button_clicked()
{
    std::cout<<"on start scenarion clicked"<<std::endl;
    if (f_cnt != nullptr) {
        for(int i=0; i < ecn.getECKnownSlavesCount(); ++i) {
            f_cnt[i].setTargetVelocity(0);
        }
        ui.line_speed1_1->setText("000.0");
        ui.line_speed1_2->setText("000.0");
        ui.line_speed1_3->setText("000.0");
        ui.line_speed1_4->setText("000.0");
        ui.line_speed1_5->setText("000.0");
        ui.line_speed1_6->setText("000.0");
    }
    gettimeofday(&clickScenario, nullptr);
    for(auto i = std::begin(instructionsList); i != std::end(instructionsList); ++i) {
        (*i)->clrScheduled();
    }
  {std::lock_guard<std::recursive_mutex> lock { instructionsMutex };
    currentStep = instructionsList.begin();
  }
}

void WindowFirst::on_reset_list_button_clicked()
{
    std::cout<<"on reset list clicked"<<std::endl;
  { std::lock_guard<std::recursive_mutex> lock { instructionsMutex};
    currentStep = instructionsList.end();
  }
    
    if (f_cnt != nullptr) {
        for(int i=0; i < ecn.getECKnownSlavesCount(); ++i) {
            f_cnt[i].setTargetVelocity(0);
        }
        ui.line_speed1_1->setText("--0.0");
        ui.line_speed1_2->setText("--0.0");
        ui.line_speed1_3->setText("--0.0");
        ui.line_speed1_4->setText("--0.0");
        ui.line_speed1_5->setText("--0.0");
        ui.line_speed1_6->setText("--0.0");
    }
}

void WindowFirst::tickNextScenarioStep()
{
  std::lock_guard<std::recursive_mutex> lock {instructionsMutex};
    if (currentStep == instructionsList.end() || (*currentStep) == nullptr)
        return;

    std::vector<bool> reachedTarget;
    for( auto i =0; i < JOINTS_COUNT && i < ecn.getDiscoveredSlavesCount(); ++i) {
        reachedTarget.push_back( f_cnt[i].isTargetReached());
    }

    struct timeval now;
    gettimeofday(&now, nullptr);
    if ((*currentStep)->isVelocityType() && 
        (clickScenario + (*currentStep)->executionMoment < now) ) {
        auto dis = std::distance(instructionsList.begin(), currentStep);
        std::cout<<"Setting requirement of step "<<dis<<std::endl;
        for( auto const &i: (*currentStep)->listOfMotorIDX_velocity) {
        std::cout<<"\t\t command for "<<i.first<<"  with vel "<<
        i.second<<std::endl;
            if ( i.first > 0 && i.first <= ecn.getECKnownSlavesCount()) {
                StartTimers[i.first - 1] = {0, 0};
                f_cnt[i.first - 1].setTargetVelocity( i.second);
                auto v = QString("%1").arg( 60. * i.second);
                switch (i.first) {
                    case 1:
                        ui.line_speed1_1->setText(v);
                        break;
                    case 2:
                        ui.line_speed1_2->setText(v);
                        break;
                    case 3:
                        ui.line_speed1_3->setText(v);
                        break;
                    case 4:
                        ui.line_speed1_4->setText(v);
                        break;
                    case 5:
                        ui.line_speed1_5->setText(v);
                        break;
                    case 6:
                        ui.line_speed1_6->setText(v);
                        break;
                }
            }
        }
        ++currentStep;
        ui.step_num->setText(QString("%1").arg(dis + 1));
    } else
    if ( (*currentStep)->isPositionalType()) {
        auto dis = std::distance(instructionsList.begin(), currentStep);
        if ((*currentStep)->isScheduled() && (*currentStep)->isDone(reachedTarget)) {
            std::cout<<"Setting step for execution "<<dis + 1<<std::endl;
            std::cout<<"  which occured at position:"<<std::endl;
            for(auto i = 0; i < ecn.getDiscoveredSlavesCount() && i < JOINTS_COUNT ; ++i){
                std::cout<<i<<"   -> "<<f_cnt[i].getPositionValue()<<std::endl;
            }
            ++currentStep;
            ui.step_num->setText(QString("%1").arg(dis + 1));
        } else
        if (!(*currentStep)->isScheduled()) {
            std::cout<<"Scheduling step "<<dis <<" at position:"<<std::endl;
            for(auto i = 0; i < ecn.getDiscoveredSlavesCount() && i < JOINTS_COUNT ; ++i){
                std::cout<<i<<"   -> "<<f_cnt[i].getPositionValue()<<std::endl;
            }
            for( auto const & i: (*currentStep)->listMotorPositionSpeedEndSpeed) {
                if(i.idx < 1 || i.idx > ecn.getECKnownSlavesCount()) {
                    continue;
                }
                f_cnt[i.idx - 1].setTargetPosition(i.targetPosition);
                f_cnt[i.idx - 1].setPositioningVelocity(i.targetVel);
                f_cnt[i.idx -1].setEndVelocity(i.targetEndVelocity);
                f_cnt[i.idx - 1].startPositionChange();
            }
            (*currentStep)->setScheduled();
        }
    } else
    if ( (*currentStep)->isDelayType() ) {
        if ((*currentStep)->isScheduled()) {
            if ( (*currentStep)->tv + (*currentStep)->delayDuration < now ) {
                std::cout<<"delay is complete. Switching next "<<std::endl;
                ++currentStep;
            }
        } else {
            (*currentStep)->tv = now;
            (*currentStep)->setScheduled();
        }
    }
    if (currentStep == instructionsList.end()) {
        on_reset_list_button_clicked();
    }
}

void WindowFirst::createLayoutWidgets()
{
    modesLineEdits = new QLineEdit[6];
    statusLineEdits = new QLineEdit[6];
    positionLineEdits = new QLineEdit[6];
    errorLineEdits = new QLineEdit[6];
    velocityLineEdits = new QLineEdit[6];
    currentLineEdits = new QLineEdit[6];
    warningLineEdits = new QLineEdit[6];
    digiInputLineEdits = new QLineEdit[6];
    for(auto i =5; i > -1; --i)
    {
        modesLineEdits[i].setParent(this);
        statusLineEdits[i].setParent(this);
        positionLineEdits[i].setParent(this);
        errorLineEdits[i].setParent(this);
        velocityLineEdits[i].setParent(this);
        currentLineEdits[i].setParent(this);
        warningLineEdits[i].setParent(this);
        digiInputLineEdits[i].setParent(this);

        modesLineEdits[i].setEnabled(false);
        statusLineEdits[i].setEnabled(false);
        positionLineEdits[i].setEnabled(false);
        errorLineEdits[i].setEnabled(false);
        velocityLineEdits[i].setEnabled(false);
        currentLineEdits[i].setEnabled(false);
        warningLineEdits[i].setEnabled(false);
        digiInputLineEdits[i].setEnabled(false);

        modesLineEdits[i].setText(QString("mode %1").arg(i));
        statusLineEdits[i].setText(QString("status %1").arg(i));
        positionLineEdits[i].setText(QString("position %1").arg(i));
        errorLineEdits[i].setText(QString("error %1").arg(i));
        velocityLineEdits[i].setText(QString("velocity %1").arg(i));
        currentLineEdits[i].setText(QString("current %1").arg(i));
        warningLineEdits[i].setText(QString("warning %1").arg(i));
        digiInputLineEdits[i].setText(QString("digi input %1").arg(i));

        ui.layout_modes->addWidget(modesLineEdits + i);
        ui.layout_status->addWidget(statusLineEdits + i);
        ui.layout_position->addWidget(positionLineEdits + i);
        ui.layout_error->addWidget(errorLineEdits + i);
        ui.layout_velocity->addWidget(velocityLineEdits + i);
        ui.layout_current->addWidget(currentLineEdits + i);
        ui.layout_warning->addWidget(warningLineEdits + i);
        ui.layout_digi_inputs->addWidget(digiInputLineEdits + i);
    }
}

void WindowFirst::destroyLayoutWidgets()
{
    delete[] modesLineEdits;
    delete[] statusLineEdits;
    delete[] positionLineEdits;
    delete[] errorLineEdits;
    delete[] velocityLineEdits;
    delete[] currentLineEdits;
    delete[] warningLineEdits;
    delete[] digiInputLineEdits;
}

void WindowFirst::reflectCyclicDataToControls()
{
    for(auto i=0; i < 6 && i < ecn.getDiscoveredSlavesCount(); ++i) {
        modesLineEdits[i].setText(QString("%1").arg(f_cnt[i].s2m.modes_of_operation_display));
        statusLineEdits[i].setText(QString("%1").arg(f_cnt[i].s2m.statusword, 0, 2));
        positionLineEdits[i].setText(QString("%1").arg(f_cnt[i].s2m.position_actual_value));
        errorLineEdits[i].setText(QString("%1").arg(f_cnt[i].s2m.error_register, 0, 2));
        velocityLineEdits[i].setText(QString("%1").arg(f_cnt[i].s2m.velocity_actual_value));
        currentLineEdits[i].setText(QString("%1").arg(f_cnt[i].s2m.current_actual_value));
        warningLineEdits[i].setText(QString("%1").arg(f_cnt[i].s2m.last_warning_code, 0, 2));
        digiInputLineEdits[i].setText(QString("%1").arg(f_cnt[i].s2m.digital_inputs, 0, 2));
    }
}

void WindowFirst::startGoToPosition(int mtr, int position, int velocity)
{
std::cout<<"startGoToPosition  called for mtr="<<mtr<<"    with position "<<position<<"   and velocity "<<velocity<<std::endl;
    if (!MotorIndexOperationAllowed(mtr)) return;
    f_cnt[mtr - 1].setTargetPosition(position);
    f_cnt[mtr - 1].setPositioningVelocity(velocity);
    f_cnt[mtr - 1].startPositionChange();
}

std::list <int> zeroValues { 0, 0, 0, 0, 0, 522720 };

void WindowFirst::on_write_zero_button_clicked()
{
    std::ofstream zeroFile(ZERO_STREAM_FILENAME);
    if (!zeroFile.is_open()) {
        // TODO notify on the problem with file opening
        return;
    }
    int idx =0;
    for(auto const & i: zeroValues) {
        zeroFile<<i<<std::endl;
        knownZeroes[idx++] =i;
    }
    zeroFile.close();
}

void WindowFirst::loadZeroPositions()
{
    std::ifstream zeroFile(ZERO_STREAM_FILENAME);
    if (!zeroFile.is_open())
    {
        int idx = 0;
        for( auto const & i: zeroValues) {
            knownZeroes[idx++] = i;
        }
        return;
    }
    for(auto i=0; i < JOINTS_COUNT; ++i) {
        zeroFile>>knownZeroes[i];
    }
    zeroFile.close();
}

void WindowFirst::on_clear_all_button_clicked()
{
    if(f_cnt == nullptr)
        return;

    for(auto i =0; i < ecn.getDiscoveredSlavesCount() && i < JOINTS_COUNT; ++i)
    {
        f_cnt[i].clearError();
    }
}

void WindowFirst::on_go_initial_button_clicked()
{
    if (f_cnt == nullptr) {
        return;
    }
    for(auto i =0; i < ecn.getDiscoveredSlavesCount() && i < JOINTS_COUNT; ++i)
    {
        f_cnt[i].setTargetPosition(-knownZeroes[i]);
        f_cnt[i].setPositioningVelocity(60000); // 1000 rpms ( 60 in multiplier for 
                                                // rpms to (0.1 deg / sec )
    }
    for(auto i=0; i < ecn.getDiscoveredSlavesCount() && i < JOINTS_COUNT; ++i)
    {
        f_cnt[i].startPositionChange();
    }
}

void WindowFirst::on_all_zero_clicked()
{
    if (f_cnt == nullptr) {
        return;
    }
    for(auto i =0; i < ecn.getDiscoveredSlavesCount() && i < JOINTS_COUNT; ++i)
    {
        f_cnt[i].setTargetPosition(0);
        f_cnt[i].setPositioningVelocity(120000); // 1000 rpms ( 60 in multiplier for 
                                                // rpms to (0.1 deg / sec )
        f_cnt[i].setEndVelocity(0);
    }
    for(auto i=0; i < ecn.getDiscoveredSlavesCount() && i < JOINTS_COUNT; ++i)
    {
        f_cnt[i].startPositionChange();
    }
}

void WindowFirst::on_open_grasp_button_pressed()
{
    ek1828->powerOnChannel(EK1828_OPEN_CHANNEL_NUMBER);
}

void WindowFirst::on_open_grasp_button_released()
{
    ek1828->powerOffChannel(EK1828_OPEN_CHANNEL_NUMBER);
}

void WindowFirst::on_close_grasp_button_pressed()
{
    ek1828->powerOnChannel(EK1828_CLOSE_CHANNEL_NUMBER);
}

void WindowFirst::on_close_grasp_button_released()
{
    ek1828->powerOffChannel(EK1828_CLOSE_CHANNEL_NUMBER);
}
