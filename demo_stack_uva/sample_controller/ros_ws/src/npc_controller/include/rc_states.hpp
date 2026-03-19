#ifndef RC_STATES_HPP
#define RC_STATES_HPP

enum class CTState
{
  CT1_PWR_ON = 1,
  CT2_INITIALIZED = 2,
  CT3_ACT_TEST = 3,
  CT4_CRANKREADY = 4,
  CT5_CRANKING = 5,
  CT6_RACEREADY = 6,
  CT7_INIT_DRIVING = 7,
  CT8_CAUTION = 8,
  CT9_NOM_RACE = 9,
  CT10_COORD_STOP = 10,
  CT11_CNTRL_SHUTDOWN = 11,
  CT12_EMRG_SHUTDOWN = 12,
  CT255_DEFAULT = 255
}; // enum class CtState

enum class SysState
{
  SS1_PWR_ON = 1,
  SS2_SUBSYS_CON = 2,
  SS3_ACT_TESTING = 3,
  SS4_ACT_TEST_DONE = 4,
  SS5_CRANKREADY = 5,
  SS6_PRECRANK_CHECK = 6,
  SS7_CRANKING = 7,
  SS8_ENG_RUNNING = 8,
  SS9_DRIVING = 9,
  SS10_SHUT_ENG = 10,
  SS11_PWR_OFF = 11,
  SS13_CRANK_CHECK_INIT = 13,
  SS19_STANDBY_MODE = 19,
  SS255_DEFAULT = 255
}; // enum class SysState

enum class Rc2TrackFlags
{
  Rc2TrackFlag_Null = 0,
  Rc2TrackFlag_Red = 3,
  Rc2TrackFlag_FCY = 9,
  Rc2TrackFlag_Green = 1,
  Rc2TrackFlag_WavingGreen = 37,
  Rc2TrackFlag_Chequered = 4
}; // enum class TrackFlag

enum class Rc2VehFlags
{
  Rc2VehFlag_Null = 0,
  Rc2VehFlag_Orange = 25,
  Rc2VehFlag_Yellow = 7,
  Rc2VehFlag_Stop = 34,
  Rc2VehFlag_Blue = 2,
  Rc2VehFlag_Black = 4,
  Rc2VehFlag_Purple = 32,
  Rc2VehFlag_EngineKill = 33,
  Rc2VehFlag_Defender = 35,
  Rc2VehFlag_Attacker = 36
}; // enum class VehicleFlag

enum class LapState
{
  LS0_IN_BOX = 0,
  LS1_LEAVE_BOX = 1,
  LS2_PIT_EXIT = 2,
  LS3_ON_RACELINE = 3,
  LS4_BLACK_SLOWDOWN = 4,
  LS5_BLACK_SWITCH_TO_PIT = 5,
  LS6_PIT_ENTRY = 6,
  LS7_TERMINATE = 7,
  LS255_DEFAULT = 255
}; // enum class LapState

enum class TrackLocation
{
  LOC_START_FINISH = 0,
  LOC_PIT_ENTRY_EXIT = 1,
  LOC_PASSING_ZONE = 2,
  LOC_PIT_SPEEDUP = 10,
  LOC_PIT_SLOWDOWN = 11,
  LOC_DEFAULT = 255
}; // enum class TrackLocation

CTState int2ct(int ct_state)
{
  switch (ct_state)
  {
  case 1:
    return CTState::CT1_PWR_ON;
  case 2:
    return CTState::CT2_INITIALIZED;
  case 3:
    return CTState::CT3_ACT_TEST;
  case 4:
    return CTState::CT4_CRANKREADY;
  case 5:
    return CTState::CT5_CRANKING;
  case 6:
    return CTState::CT6_RACEREADY;
  case 7:
    return CTState::CT7_INIT_DRIVING;
  case 8:
    return CTState::CT8_CAUTION;
  case 9:
    return CTState::CT9_NOM_RACE;
  case 10:
    return CTState::CT10_COORD_STOP;
  case 11:
    return CTState::CT11_CNTRL_SHUTDOWN;
  case 12:
    return CTState::CT12_EMRG_SHUTDOWN;
  case 255:
  default:
    return CTState::CT255_DEFAULT;
  }
}

SysState int2sys(int sys_state)
{
  switch (sys_state)
  {
  case 1:
    return SysState::SS1_PWR_ON;
  case 2:
    return SysState::SS2_SUBSYS_CON;
  case 3:
    return SysState::SS3_ACT_TESTING;
  case 4:
    return SysState::SS4_ACT_TEST_DONE;
  case 5:
    return SysState::SS5_CRANKREADY;
  case 6:
    return SysState::SS6_PRECRANK_CHECK;
  case 7:
    return SysState::SS7_CRANKING;
  case 8:
    return SysState::SS8_ENG_RUNNING;
  case 9:
    return SysState::SS9_DRIVING;
  case 10:
    return SysState::SS10_SHUT_ENG;
  case 11:
    return SysState::SS11_PWR_OFF;
  case 13:
    return SysState::SS13_CRANK_CHECK_INIT;
  case 19:
    return SysState::SS19_STANDBY_MODE;
  case 255:
  default:
    return SysState::SS255_DEFAULT;
  }
}

Rc2TrackFlags int2tf(int track_flag)
{
  switch (track_flag)
  {
  case 0:
    return Rc2TrackFlags::Rc2TrackFlag_Null;
  case 3:
    return Rc2TrackFlags::Rc2TrackFlag_Red;
  case 9:
    return Rc2TrackFlags::Rc2TrackFlag_FCY;
  case 1:
    return Rc2TrackFlags::Rc2TrackFlag_Green;
  case 37:
    return Rc2TrackFlags::Rc2TrackFlag_WavingGreen;
  case 4:
    return Rc2TrackFlags::Rc2TrackFlag_Chequered;

  default:
    return Rc2TrackFlags::Rc2TrackFlag_Null;
  }
}

Rc2VehFlags int2vf(int veh_flag)
{
  switch (veh_flag)
  {
  case 0:
    return Rc2VehFlags::Rc2VehFlag_Null;
  case 25:
    return Rc2VehFlags::Rc2VehFlag_Orange;
  case 7:
    return Rc2VehFlags::Rc2VehFlag_Yellow;
  case 34:
    return Rc2VehFlags::Rc2VehFlag_Stop;
  case 2:
    return Rc2VehFlags::Rc2VehFlag_Blue;
  case 4:
    return Rc2VehFlags::Rc2VehFlag_Black;
  case 32:
    return Rc2VehFlags::Rc2VehFlag_Purple;
  case 33:
    return Rc2VehFlags::Rc2VehFlag_EngineKill;
  case 35:
    return Rc2VehFlags::Rc2VehFlag_Defender;
  case 36:
    return Rc2VehFlags::Rc2VehFlag_Attacker;

  default:
    return Rc2VehFlags::Rc2VehFlag_Null;
  }
}

LapState int2lap(int lap_state)
{
  switch (lap_state)
  {
  case 0:
    return LapState::LS0_IN_BOX;
  case 1:
    return LapState::LS1_LEAVE_BOX;
  case 2:
    return LapState::LS2_PIT_EXIT;
  case 3:
    return LapState::LS3_ON_RACELINE;
  case 4:
    return LapState::LS4_BLACK_SLOWDOWN;
  case 5:
    return LapState::LS5_BLACK_SWITCH_TO_PIT;
  case 6:
    return LapState::LS6_PIT_ENTRY;
  case 7:
    return LapState::LS7_TERMINATE;
  case 255:
  default:
    return LapState::LS255_DEFAULT;
  }
}

TrackLocation int2loc(int lap_location)
{
  switch (lap_location)
  {
  case 0:
    return TrackLocation::LOC_START_FINISH;
  case 1:
    return TrackLocation::LOC_PIT_ENTRY_EXIT;
  case 2:
    return TrackLocation::LOC_PASSING_ZONE;
  case 10:
    return TrackLocation::LOC_PIT_SPEEDUP;
  case 11:
    return TrackLocation::LOC_PIT_SLOWDOWN;
  case 255:
  default:
    return TrackLocation::LOC_DEFAULT;
  }
}

#endif // RC_STATES_HPP
