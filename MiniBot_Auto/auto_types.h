// Types for MiniBot_Auto.
// These live in a header because the Arduino build inserts auto-generated
// function prototypes above everything in the .ino, so a type defined in the
// .ino itself is not visible to them.

#pragma once
#include <Arduino.h>

// ------------------------------------------------------------------- motors
enum MotorType { MG90_CR, MG90_Degree, N20Plus, STD_SERVO, Victor_SPX, Talon_SRX, SPARK_MAX, CUSTOM };

struct MotorTypeConfig {
  MotorType type;
  float pwmFreq;
  int   minPulseUs;
  int   maxPulseUs;
};

// ------------------------------------------------------------------ geometry
struct Pose2D {
  int16_t x_mm;
  int16_t y_mm;
  float   heading_deg;
};

// Target ids the laptop can send.
// Each alliance owns 2 corner platforms plus the centre platform on its
// boundary - 3 per side, which is what the field photo shows.
enum TargetId : uint8_t {
  TGT_NONE          = 0,
  TGT_CORNER_BLUE_A = 1,   // blue zone, driver-left
  TGT_CORNER_BLUE_B = 2,   // blue zone, driver-right
  TGT_CORNER_ORG_A  = 3,
  TGT_CORNER_ORG_B  = 4,
  TGT_CENTER_BLUE   = 5,
  TGT_CENTER_ORANGE = 6,
  TGT_COUNT
};

struct RampApproach {
  Pose2D staging;        // flat ground, lined up with the ramp
  float  climb_heading;  // heading to hold all the way up
};

struct CenterPlatform {
  Pose2D       center;
  RampApproach ramps[3];
};

// A circular keep-out we must not cut through on the way somewhere else.
struct Obstacle {
  int16_t x_mm;
  int16_t y_mm;
  int16_t radius_mm;
};

// --------------------------------------------------------------------- link
#define LINK_MAGIC   0x5A
#define LINK_VERSION 1

enum LinkCmd : uint8_t {
  CMD_NONE     = 0,
  CMD_GOTO_XY  = 1,   // drive to gx,gy then hold
  CMD_GOTO_TGT = 2,   // drive to targetId using its approach routine
  CMD_CANCEL   = 3,   // abandon current goal, stop, stay enabled
  CMD_ENABLE   = 4,   // the one start action on the field signal
  CMD_DISABLE  = 5,   // match end / e-stop
};

#pragma pack(push, 1)
struct LinkPacket {
  uint8_t  magic;
  uint8_t  version;
  uint16_t seq;
  // pose, refreshed every camera frame
  int16_t  x_mm;
  int16_t  y_mm;
  int16_t  heading_cdeg;   // centi-degrees
  uint8_t  poseValid;      // 0 = tag not seen this frame
  // command, usually CMD_NONE
  uint8_t  cmd;
  int16_t  gx_mm;
  int16_t  gy_mm;
  uint8_t  targetId;
  uint8_t  checksum;       // XOR of every byte before this one
};
#pragma pack(pop)

// -------------------------------------------------------------------- state
enum AutoState {
  ST_DISABLED,   // boot state, and where we land on any failsafe
  ST_IDLE,       // enabled, no goal
  ST_DRIVE,      // driving to a point on flat ground
  ST_ALIGN,      // squaring up to a ramp before climbing
  ST_CLIMB,      // straight up the ramp, no steering authority
  ST_HOLD,       // sitting still for the 3 s capture
  ST_DONE,       // goal complete, waiting for the next one
};
