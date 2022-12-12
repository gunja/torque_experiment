#include <unordered_map>
#include <string>
#include <cstdlib>

#include <ncurses.h>
#include "ecn.h"
#include "festo_over_ec.h"

#define VOLTAGE_ENABLED_STATUS_MASK 0x10
#define WARNING_STATUS_MASK 0x80
#define DRIVE_MOVING_STATUS_MASK 0x100
#define REMOTE_STATUS_MASK 0x200
#define TARGET_REACHED_STATUS_MASK 0x400
#define INTERNAL_LIMIT_ACTIVE_STATUS_MASK 0x800
#define SET_POINT_ACK_STATUS_MASK 0x1000
#define FOLLO_HOM_ERR_STATUS_MASK 0x2000
#define MANUF_ST_BIT_STATUS_MASK 0x4000
#define DRIVE_ORIG_POINT_STATUS_MASK 0x8000

#define STATE_STATUS_MASK_LOW 0x4F
#define STATE_STATUS_MASK_F 0x6F

#define STATE_STATUS_NOT_READY_SW_ON_VAL 0x0
#define STATE_STATUS_SW_ON_DIS_VAL 0x40
#define STATE_STATUS_RD_SW_ON_VAL 0x21
#define STATE_STATUS_SWD_ON_VAL 0x23
#define STATE_STATUS_OPER_EN_VAL 0x27
#define STATE_STATUS_QK_STOP_ACTIVE_VAL 0x7
#define STATE_STATUS_FAULT_REACTION_ACTIVE_VAL 0xF
#define STATE_STATUS_FAULT_VAL 0x8
#define STATE_STATUS_FAULT_CIA402_VAL 0x8



void display_state( EtherCATNetwork & net, int sl);
void soem_bring_up( EtherCATNetwork &net, const char *iface);
void soem_switch_operational(EtherCATNetwork &net);
void soem_switch_safeoperational(EtherCATNetwork &ecNet);
void display_slave_info( EtherCATNetwork & net, int, FestoController *);
void display_cyclic_info( EtherCATNetwork & net, int, FestoController *);

std::unordered_map< int, std::string> displayModeOperationTexts;

void fillDMOT()
{
    displayModeOperationTexts.insert( { -1, "INVALID mode"});
    displayModeOperationTexts.insert( { -11, "User Position mode"});
    displayModeOperationTexts.insert( { -13, "User Velocity mode"});
    displayModeOperationTexts.insert( { -14, "User Torque mode"});
    displayModeOperationTexts.insert( { 1, "Profile Position mode"});
    displayModeOperationTexts.insert( { 3, "Profile Velocity mode"});
    displayModeOperationTexts.insert( { 4, "Profile Torque mode"});
    displayModeOperationTexts.insert( { 6, "Homing mode"});
    displayModeOperationTexts.insert( { 7, "Interpolated Position mode"});
}

int main()
{
    EtherCATNetwork ecNet;
    FestoController *contrls = nullptr;
    const char iface[] = "enp59s0";
    int ch;
    int activeSlave = 0;
    fillDMOT();

    initscr();
    raw();
    noecho();
    keypad(stdscr, true);
    halfdelay(1);

    while(true)
    {
        ch = getch();
        switch(ch) {
            case ERR:
                break;
            case 'B': case 'b':
                // bring up SOEM
                soem_bring_up(ecNet, iface);
                break;
            case 'I' : case 'i':
                soem_switch_operational(ecNet);
                clear();
                break;
            case 's': case 'S':
                soem_switch_safeoperational(ecNet);
                clear();
                break;

            case KEY_UP:
                activeSlave--;
                if (activeSlave <=0) activeSlave = 0;
                break;
            case KEY_DOWN:
                activeSlave++;
                if (activeSlave > ecNet.getECKnownSlavesCount())
                    activeSlave = ecNet.getECKnownSlavesCount();
                break;
            case KEY_RIGHT:
                if (activeSlave > 0 && activeSlave <= ecNet.getECKnownSlavesCount()) {
                    contrls[activeSlave - 1].m2s.target_velocity += 1000;
                }
                break;
            case KEY_LEFT:
                if (activeSlave > 0 && activeSlave <= ecNet.getECKnownSlavesCount()) {
                    contrls[activeSlave - 1].m2s.target_velocity -= 1000;
                }
                break;
            case 'e': case 'E':
                if (ecNet.isOperational() && contrls != nullptr && activeSlave > 0
                    && activeSlave <= ecNet.getECKnownSlavesCount()) {
                    contrls[activeSlave - 1].startEnabling();
                } else { ecNet.enableVoltage(activeSlave); }
                break;
            case 'c': case 'C':
                if (ecNet.isOperational() && contrls != nullptr && activeSlave > 0
                    && activeSlave <= ecNet.getECKnownSlavesCount()) {
                    contrls[activeSlave - 1].clearError();
                } else {
                    ecNet.clearErrorState(activeSlave);
                }
                break;
            case 'd': case 'D':
                if (ecNet.isOperational() && contrls != nullptr && activeSlave > 0
                    && activeSlave <= ecNet.getECKnownSlavesCount()) {
                    contrls[activeSlave - 1].disableMotor();
                } else {
                    ecNet.disableMotorASYNC(activeSlave);
                }
                break;
            case 'h': case 'H':
                if (ecNet.isOperational()) {
                    contrls[activeSlave - 1].toggleHalt();
                }
                break;
            case 'j':
                ecNet.setHome(activeSlave);
                break;
            case 'v':
                ecNet.requestProfile(activeSlave, 3);
                break;
            case 'o':
                ecNet.setMaskUINT32( activeSlave, 0x60FE, 0x02, 0xBFF0000U);
                break;
            case 'p':
                ecNet.clearMaskUINT32(activeSlave, 0x60FE, 0x02, 0xBFF0000U);
                break;
            case 'q':
                ecNet.setMaskUINT32(activeSlave, 0x60FE, 0x02, 0x3000000U);
                break;
            case 'w':
                ecNet.clearMaskUINT32(activeSlave, 0x60FE, 0x02, 0x3000000U);
                break;
            default:
                break;
        }
        if (ecNet.isSafeOperational() && contrls == nullptr ) {
            contrls = reinterpret_cast<FestoController*>(std::calloc(
                        ecNet.getECKnownSlavesCount(), sizeof(FestoController)));
            for(int i = 0; i < ecNet.getECKnownSlavesCount(); ++i) {
                new (&contrls[i]) FestoController(
                            *(reinterpret_cast<Master2Slave*>(ecNet.getOutputPointer(i+1))),
                            *(reinterpret_cast<Slave2Master*>(ecNet.getInputPointer(i+1))),
                             ecNet, i+1);
            }
        }
        if (contrls != nullptr ) {
            for(int i=0 ; i < ecNet.getECKnownSlavesCount(); ++i ) {
                contrls[i].updateOutputs();
            }
        }
        //ecNet.exchangeData();
        clear();
        if (activeSlave > 0) {
            display_slave_info(ecNet, activeSlave, contrls);
        }
        display_state(ecNet, activeSlave);
        refresh();
    }

    endwin();
    return 0;
}

void soem_bring_up( EtherCATNetwork &net, const char *iface)
{
    net.startNetwork(iface);
}


void display_state( EtherCATNetwork & net, int slNum)
{
    if( !net.networkRunning())
    {
        wmove(stdscr, 10, 10);
        printw("Network is not running");
        wmove(stdscr, 11, 10);
        printw("Press B or b to start network");

        if(net.hasError()) {
            wmove( stdscr, 10, 13);
            printw("Last reported Error %s", net.getLastErrorText());
        }
        return;
    }

    wmove(stdscr, 1, 5);
    printw("EtherCAT network initialized with %d / %d slaves ", net.getDiscoveredSlavesCount(),
                net.getECKnownSlavesCount());
    printw(" Network state is %d and isSafeOperational is %d", net.getNetState(),
            net.isSafeOperational());
    wmove(stdscr, 3, 6);
    printw("Press s  or S to switch network to SAFE-OP state");
    wmove(stdscr, 4, 6);
    printw("Press i  or I to switch network to OP state");
    printw(" Press e or E to Enable motor of selected slave");
    printw(" Press c or C to clear errors of selected controller");

    for(int i=1; i <= net.getECKnownSlavesCount(); ++i)
    {
        wmove(stdscr, 6+ 2 *i, 6);
        printw("%s %2d -> %s status=0x%04X",
           ( i == slNum )? ">": " ",
            i, net.getSlaveName(i),
            net.getSlotDataUINT16(i, 0x6041, 0x00));
        wmove(stdscr, 6+ 2 *i + 1, 6);
        printw("1C00.0 = %u| .1 = %u| .2 = %u| .3 = %u| .4 = %u",
                net.getSlotDataUINT8(i, 0x1C00, 0),
                net.getSlotDataUINT8(i, 0x1C00, 1),
                net.getSlotDataUINT8(i, 0x1C00, 2),
                net.getSlotDataUINT8(i, 0x1C00, 3),
                net.getSlotDataUINT8(i, 0x1C00, 4)
            );
    }
}

void soem_switch_operational(EtherCATNetwork &net)
{
    net.SwitchOperational();
}

void write_portion( int &lineNum, EtherCATNetwork &net, int s, int startCol, int slot)
{
    wmove(stdscr, lineNum, startCol); printw("%04X [PDO slave-> master]: count: %u", slot,
            net.getSlotDataUINT8(s, slot, 0));
    ++lineNum;
    wmove(stdscr, lineNum, startCol + 5);
    printw("%04X.1 -> 0x%08X", slot, net.getSlotDataUINT32(s, slot, 1));
    ++lineNum;
    wmove(stdscr, lineNum, startCol + 5);
    printw("%04X.2 -> 0x%08X", slot, net.getSlotDataUINT32(s, slot, 2));
    ++lineNum;
    wmove(stdscr, lineNum, startCol + 5);
    printw("%04X.3 -> 0x%08X", slot, net.getSlotDataUINT32(s, slot, 3));
    ++lineNum;
    wmove(stdscr, lineNum, startCol + 5);
    printw("%04X.4 -> 0x%08X", slot, net.getSlotDataUINT32(s, slot, 4));
}

void display_slave_info( EtherCATNetwork & net, int s, FestoController * cnt)
{
    if (!net.networkRunning()) return;
    if (net.isOperational()) {
        display_cyclic_info(net, s, cnt);
        return;
    }

    int lineNum = 5;
    int ln;

    wmove(stdscr, lineNum, 70);
    printw("Slave #%02d", s);
    ++lineNum; ++lineNum;
    wmove(stdscr, lineNum, 60);
    int c = net.getSlaveState(s);
    printw("Slave State is: %d (%s)  with status = 0x%04X  and control = 0x%04X"
        " |  mode_rq=%d mode_disp=%d | digiFlag =0x%08X", c,
        net.ecStateMachineAsText(c), net.getSlotDataUINT16(s, 0x6041, 0x00),
        net.getSlotDataUINT16(s, 0x6040, 0x00)
        , net.getSlotDataUINT8(s, 0x6060, 0x00),  net.getSlotDataUINT8(s, 0x6061, 0x00)
        , net.getSlotDataUINT32(s, 0x60FE, 2)
	);

    ++lineNum;
    wmove( stdscr, lineNum, 60);
    printw("PositionFactor: %7u / %7u", net.getSlotDataUINT32(s, 0x6093, 0x01),
                net.getSlotDataUINT32(s, 0x6093, 0x02));
    ++lineNum;
    wmove( stdscr, lineNum, 60);
    printw("VelocityFactor: %7u / %7u", net.getSlotDataUINT32(s, 0x6094, 0x01),
                net.getSlotDataUINT32(s, 0x6094, 0x02));
    ++lineNum;
    wmove( stdscr, lineNum, 60);
    printw("AccelerationFactor: %7u / %7u", net.getSlotDataUINT32(s, 0x6097, 0x01),
                net.getSlotDataUINT32(s, 0x6097, 0x02));

    ++lineNum; ++lineNum;
    int SLOT = 0x1C12;
    write_portion(lineNum, net, s, 60, SLOT);
    SLOT = 0x1C13;
    write_portion(lineNum, net, s, 60, SLOT);
    ++lineNum; ++lineNum;
    SLOT = 0x1600;
    ln = lineNum;
    write_portion(ln, net, s, 60, SLOT);
    SLOT = 0x1602;
    write_portion(lineNum, net, s, 100, SLOT);
    ++lineNum; ++lineNum;
    SLOT = 0x1601;
    ln = lineNum;
    write_portion(lineNum, net, s, 60, SLOT);
    write_portion(ln, net, s, 100, 0x1603);

    ++lineNum; ++lineNum;
    ln = lineNum;
    SLOT = 0x1A00;
    write_portion( ln, net, s, 60, SLOT);
    SLOT = 0x1A02;
    write_portion( lineNum, net, s, 100, SLOT);
    ++lineNum; ++lineNum;
    SLOT = 0x1A01;
    ln = lineNum;
    write_portion( ln, net, s, 60, SLOT);
    SLOT = 0x1A03;
    write_portion( lineNum, net, s, 100, SLOT);
    ++lineNum; ++lineNum;
    SLOT = 0x6061;
    wmove(stdscr, lineNum, 60); printw("%04X [Mode of Oper Disp]: count: %u", SLOT,
            net.getSlotDataUINT8(s, SLOT, 0));
}

void soem_switch_safeoperational(EtherCATNetwork &ecNet)
{
    ecNet.SwitchSafeOperational();
}

const char * getTextForDisplayMode( int v)
{
    auto r = displayModeOperationTexts.find(v);
    if (r == displayModeOperationTexts.end()) {
        return "was not found";
    }
    return r->second.c_str();
}

const char *decodeStatusWord(uint32_t status)
{
    std::string output;
    uint32_t statAndLowmask = status &STATE_STATUS_MASK_LOW;
    uint32_t statAndStatemask = status &STATE_STATUS_MASK_F;

    if( statAndLowmask == STATE_STATUS_NOT_READY_SW_ON_VAL) {
        output += "status not ready;";
    }
    if (statAndLowmask == STATE_STATUS_SW_ON_DIS_VAL) {
        output += "switch on disabled;";
    }
    if (statAndStatemask == STATE_STATUS_RD_SW_ON_VAL) {
        output += "ready to switch on;";
    }
    if (statAndStatemask == STATE_STATUS_SWD_ON_VAL) {
        output += "switched on;";
    }
    if (statAndStatemask == STATE_STATUS_OPER_EN_VAL) {
        output += "operation enabled;";
    }
    if (statAndStatemask == STATE_STATUS_QK_STOP_ACTIVE_VAL) {
        output += "quick stop active;";
    }
    if( statAndLowmask == STATE_STATUS_FAULT_REACTION_ACTIVE_VAL) {
        output += "fault reaction active;";
    }
    if (statAndLowmask == STATE_STATUS_FAULT_VAL) {
        output += "fault state;";
    }
    if (statAndLowmask == STATE_STATUS_FAULT_CIA402_VAL) {
        output += "fault by CiA 402;";
    }

    if ( status & VOLTAGE_ENABLED_STATUS_MASK) {
        output += "Voltage enabled;";
    } else { output += "voltage DISABLED;";}
    if( status & WARNING_STATUS_MASK) {
        output += "Warning;";
    } else { output += ";"; }
    if( status & DRIVE_MOVING_STATUS_MASK) {
        output += "Drive is moving;";
    } else { output += "srive stopped;"; }
    if( status & REMOTE_STATUS_MASK) {
        output += "remote flagged;";
    } else { output += ";"; }
    if( status & TARGET_REACHED_STATUS_MASK) {
        output += "target reached;";
    } else { output+= ";"; }
    if ( status & INTERNAL_LIMIT_ACTIVE_STATUS_MASK) {
        output += "Internal LIMIT active;";
    } else { output += ";";};
    if (status & SET_POINT_ACK_STATUS_MASK) {
        output += "Set Point ACKnoledged;";
    } else { output += "no SP ack;";}
    if( status & FOLLO_HOM_ERR_STATUS_MASK) {
        output += "Follow ot Home error;";
    } else { output += ";"; }
    if (status & MANUF_ST_BIT_STATUS_MASK) {
        output += "Manufacture status set;";
    } else { output += ";"; }
    if (status & DRIVE_ORIG_POINT_STATUS_MASK) {
        output += "Drive origin reached;";
    } else { output += ";"; }

    return output.c_str();
}

void display_cyclic_info( EtherCATNetwork & net, int s, FestoController *cnt)
{
    Master2Slave m2s;
    Slave2Master s2m;
    // TODO this should be modfied later
    m2s.modes_of_operation = 3;
    m2s.controlword = 0;
    m2s.target_velocity = 0;
    m2s.end_velocity = 0;

    //net.fillExchangeRegions(s, m2s, s2m, true);

    --s;
    int ln = 10;
    int col = 60;
    wmove(stdscr, ln, col);
    printw("Mode of operation (disp) %d | %d -> %s",
        cnt[s].s2m.modes_of_operation_display,
        net.getSlotDataUINT8( s+ 1, MODE_OF_OPERATION_SLOT + 1, 0),
        getTextForDisplayMode(cnt[s].s2m.modes_of_operation_display)
    );
    printw("    Mode op op (sent) %d | %d ", cnt[s].m2s.modes_of_operation,
           net.getSlotDataUINT8( s+ 1, MODE_OF_OPERATION_SLOT, 0)
        );
    ln++;
    wmove(stdscr, ln, col);
    printw("Status word = 0x%08X ", cnt[s].s2m.statusword);
    printw("   Control wrd = 0x%08X", cnt[s].m2s.controlword);
    ln++;
    wmove(stdscr, ln, col + 5);
    printw("%s", decodeStatusWord(cnt[s].s2m.statusword));
    ln++;
    wmove(stdscr, ln, col + 5);
    printw("position actual: %d", cnt[s].s2m.position_actual_value);
    ln++;
    wmove(stdscr, ln, col + 5);
    printw("Error register content: %d", cnt[s].s2m.error_register);
    ln++;
    wmove(stdscr, ln, col + 5);
    printw("Velocity actual: %d  %d ",
        cnt[s].s2m.velocity_actual_value,
        int32_t(net.getSlotDataUINT32(s+1, 0x606C, 0)) );
    ln++;
    wmove(stdscr, ln, col + 5);
    printw("End Velocity rq: %d  %d ",
        cnt[s].m2s.end_velocity,
        net.getSlotDataUINT32(s+1, 0x6082, 0) );
    ln++;
    wmove(stdscr, ln, col + 5);
    printw("Velocity requested: %d", cnt[s].m2s.target_velocity);
    ln++;
    wmove(stdscr, ln, col + 5);
    printw("Transaction pending: %d enable %d  enInProg %d",
        cnt[s].pendingTransition, cnt[s].enabled, cnt[s].enableInProgress);
}

