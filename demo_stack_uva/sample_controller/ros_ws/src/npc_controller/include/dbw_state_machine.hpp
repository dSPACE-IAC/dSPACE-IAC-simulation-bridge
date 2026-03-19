#ifndef NPC_CONTROLLER__DBW_STATE_MACHINE_HPP_
#define NPC_CONTROLLER__DBW_STATE_MACHINE_HPP_

#include "rc_states.hpp"
namespace controller
{
    CTState dbw_state_machine(CTState &ct_state, Rc2TrackFlags &track_flag, Rc2VehFlags &vehicle_flag_, SysState &sys_state, bool estop_, int &ct_input, double &velocity)
    {
        switch (ct_state)
        {

        case CTState::CT255_DEFAULT: // CT255
            if (track_flag == Rc2TrackFlags::Rc2TrackFlag_Red)
            {
                ct_state = CTState::CT1_PWR_ON;
            }
            break;

        case CTState::CT1_PWR_ON: // CT1
            if (estop_)
            {
                ct_state = CTState::CT12_EMRG_SHUTDOWN;
            }
            else
            {
                ct_state = CTState::CT2_INITIALIZED;
            }
            break;

        case CTState::CT2_INITIALIZED: // CT2
            if (estop_)
            {
                ct_state = CTState::CT12_EMRG_SHUTDOWN;
            }
            else if (track_flag == Rc2TrackFlags::Rc2TrackFlag_Red)
            {
                ct_state = CTState::CT3_ACT_TEST;
            }
            break;

        case CTState::CT3_ACT_TEST: // CT3
            if (estop_)
            {
                ct_state = CTState::CT12_EMRG_SHUTDOWN;
            }
            else if (track_flag == Rc2TrackFlags::Rc2TrackFlag_Red)
            {
                ct_state = CTState::CT4_CRANKREADY;
            }
            break;

        case CTState::CT4_CRANKREADY: // CT4
            if (estop_)
            {
                ct_state = CTState::CT12_EMRG_SHUTDOWN;
            }
            else if (vehicle_flag_ == Rc2VehFlags::Rc2VehFlag_Orange)
            {
                ct_state = CTState::CT5_CRANKING;
            }
            break;

        case CTState::CT5_CRANKING: // CT5
            if (estop_)
            {
                ct_state = CTState::CT12_EMRG_SHUTDOWN;
            }
            else if (ct_input == 11 && velocity <= 1.0)
            {
                ct_state = CTState::CT11_CNTRL_SHUTDOWN;
            }
            else if (sys_state == SysState::SS8_ENG_RUNNING)
            {
                ct_state = CTState::CT6_RACEREADY;
            }
            break;

        case CTState::CT6_RACEREADY: // CT6
            if (estop_)
            {
                ct_state = CTState::CT12_EMRG_SHUTDOWN;
            }
            else if (ct_input == 11 && velocity <= 1.0)
            {
                ct_state = CTState::CT11_CNTRL_SHUTDOWN;
            }
            else
            {
                ct_state = CTState::CT7_INIT_DRIVING;
            }
            break;

        case CTState::CT7_INIT_DRIVING: // CT7
            if (estop_)
            {
                ct_state = CTState::CT12_EMRG_SHUTDOWN;
            }
            else if (ct_input == 11 && velocity <= 1.0)
            {
                ct_state = CTState::CT11_CNTRL_SHUTDOWN;
            }
            else if (sys_state == SysState::SS9_DRIVING && (track_flag == Rc2TrackFlags::Rc2TrackFlag_FCY || vehicle_flag_ == Rc2VehFlags::Rc2VehFlag_Yellow))
            {
                ct_state = CTState::CT8_CAUTION;
            }
            break;

        case CTState::CT8_CAUTION: // CT8
            if (estop_)
            {
                ct_state = CTState::CT12_EMRG_SHUTDOWN;
            }
            else if (ct_input == 11 && velocity <= 1.0)
            {
                ct_state = CTState::CT11_CNTRL_SHUTDOWN;
            }
            else if (vehicle_flag_ == Rc2VehFlags::Rc2VehFlag_Purple)
            {
                ct_state = CTState::CT12_EMRG_SHUTDOWN;
            }
            else if (track_flag == Rc2TrackFlags::Rc2TrackFlag_Green)
            {
                ct_state = CTState::CT9_NOM_RACE;
            }
            else if (track_flag == Rc2TrackFlags::Rc2TrackFlag_Red)
            {
                ct_state = CTState::CT10_COORD_STOP;
            }
            break;

        case CTState::CT9_NOM_RACE: // CT9
            if (estop_)
            {
                ct_state = CTState::CT12_EMRG_SHUTDOWN;
            }
            else if (ct_input == 11 && velocity <= 1.0)
            {
                ct_state = CTState::CT11_CNTRL_SHUTDOWN;
            }
            else if (vehicle_flag_ == Rc2VehFlags::Rc2VehFlag_Purple)
            {
                ct_state = CTState::CT12_EMRG_SHUTDOWN;
            }
            else if (track_flag == Rc2TrackFlags::Rc2TrackFlag_FCY || vehicle_flag_ == Rc2VehFlags::Rc2VehFlag_Yellow)
            {
                ct_state = CTState::CT8_CAUTION;
            }
            else if (track_flag == Rc2TrackFlags::Rc2TrackFlag_Red)
            {
                ct_state = CTState::CT10_COORD_STOP;
            }
            break;

        case CTState::CT10_COORD_STOP: // CT10
            if (estop_)
            {
                ct_state = CTState::CT12_EMRG_SHUTDOWN;
            }
            else if (ct_input == 11 && velocity <= 1.0)
            {
                ct_state = CTState::CT11_CNTRL_SHUTDOWN;
            }
            else if (vehicle_flag_ == Rc2VehFlags::Rc2VehFlag_Purple)
            {
                ct_state = CTState::CT12_EMRG_SHUTDOWN;
            }
            else if (track_flag == Rc2TrackFlags::Rc2TrackFlag_FCY || vehicle_flag_ == Rc2VehFlags::Rc2VehFlag_Yellow)
            {
                ct_state = CTState::CT8_CAUTION;
            }
            else if (track_flag == Rc2TrackFlags::Rc2TrackFlag_Green)
            {
                ct_state = CTState::CT9_NOM_RACE;
            }
            break;

        case CTState::CT11_CNTRL_SHUTDOWN: // CT11
            if (vehicle_flag_ == Rc2VehFlags::Rc2VehFlag_Orange)
            {
                ct_state = CTState::CT5_CRANKING;
            }
            break;

        case CTState::CT12_EMRG_SHUTDOWN: // CT12
            break;
        }

        if (ct_input != 0)
        {
            ct_input = 0;
        }
        return ct_state;
    }

} // namespace controller

#endif // NPC_CONTROLLER__DBW_STATE_MACHINE_HPP_