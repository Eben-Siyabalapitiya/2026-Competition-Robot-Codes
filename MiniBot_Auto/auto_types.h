
#pragma once
#include <Arduino.h>

enum MotorType { MG90_CR, MG90_Degree, N20Plus, STD_SERVO, Victor_SPX, Talon_SRX, SPARK_MAX, CUSTOM };

struct MotorTypeConfig {
  MotorType type;
  float pwmFreq;
  int   minPulseUs;
  int   maxPulseUs;
};

struct Pose2D {
  int16_t x_mm;
  int16_t y_mm;
  float   heading_deg;
};

enum TargetId : uint8_t {
  TGT_NONE          = 0,
  TGT_CORNER_BLUE_A = 1,
  TGT_CORNER_BLUE_B = 2,
  TGT_CORNER_ORG_A  = 3,
  TGT_CORNER_ORG_B  = 4,
  TGT_CENTER_BLUE   = 5,
  TGT_CENTER_ORANGE = 6,
  TGT_COUNT
};

struct RampApproach {
  Pose2D staging;
  float  climb_heading;
};

struct CenterPlatform {
  Pose2D       center;
  RampApproach ramps[3];
};

struct Obstacle {
  int16_t x_mm;
  int16_t y_mm;
  int16_t radius_mm;
};

#define LINK_MAGIC   0x5A
#define LINK_VERSION 1

enum LinkCmd : uint8_t {
  CMD_NONE     = 0,
  CMD_GOTO_XY  = 1,
  CMD_GOTO_TGT = 2,
  CMD_CANCEL   = 3,
  CMD_ENABLE   = 4,
  CMD_DISABLE  = 5,
};

#pragma pack(push, 1)
struct LinkPacket {
  uint8_t  magic;
  uint8_t  version;
  uint16_t seq;
  int16_t  x_mm;
  int16_t  y_mm;
  int16_t  heading_cdeg;
  uint8_t  poseValid;
  uint8_t  cmd;
  int16_t  gx_mm;
  int16_t  gy_mm;
  uint8_t  targetId;
  uint8_t  checksum;
};
#pragma pack(pop)

enum AutoState {
  ST_DISABLED,
  ST_IDLE,
  ST_DRIVE,
  ST_ALIGN,
  ST_CLIMB,
  ST_HOLD,
  ST_DONE,
};
