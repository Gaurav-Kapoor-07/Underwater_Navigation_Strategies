#pragma once

#include <cstdint>

namespace monsun_comm {

/// The packet version number is a timestamp in the format YYMMDDPP:
/// YY: Year
/// MM: Month
/// DD: Day
/// PP: patch level for intra-day updates, starts at 00
enum MonsunCommVers { MN_COMM_VERS = 20012000 };

struct MsgHeader {
    uint32_t version = MN_COMM_VERS;
    uint32_t msgType = 0;
    uint32_t source = 0;
    uint32_t relais = 0;
    uint32_t origin = 0;
};

enum MessageType {
    EmptyMsg = 0,
    DataTypesTest,

    XsensImu,
    XsensPosition,
    XsensTime,

    Ms5803Temp,
    Ms5803Press,
    Ms5803Depth,

    // GUI
    GpsWaypoint,
    GpsPath,
    // FormationMsg

    Bat7v4,
    Bat11v1,

    // GUI
    GpsArea,

    // heading_ctrl
    HeadingCtrlHeading,
    HeadingCtrlSpeed,
};

enum RelaisType { RelaisNone = 0, RelaisWifi, RelaisEvo, RelaisAhoi };

} // namespace monsun_comm
