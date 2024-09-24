/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#ifndef _UNITREE_LEGGED_COMM_H_
#define _UNITREE_LEGGED_COMM_H_

#include <stdint.h>
#include <array>

namespace UNITREE_LEGGED_SDK
{

  constexpr int HIGHLEVEL = 0xee;
  constexpr int LOWLEVEL = 0xff;
  constexpr int TRIGERLEVEL = 0xf0;
  constexpr double PosStopF = (2.146E+9f);
  constexpr double VelStopF = (16000.0f);
  extern const int HIGH_CMD_LENGTH;   // sizeof(HighCmd)
  extern const int HIGH_STATE_LENGTH; // sizeof(HighState)
  extern const int LOW_CMD_LENGTH;    // shorter than sizeof(LowCmd),   bytes compressed LowCmd length
  extern const int LOW_STATE_LENGTH;  // shorter than sizeof(LowState), bytes compressed LowState length

#pragma pack(1)

  typedef struct
  {
    uint8_t off; // set 0xA5 to turn off the battery, please try it under the premise of ensuring safety
    std::array<uint8_t, 3> reserve;
  } BmsCmd;

  typedef struct
  {
    uint8_t version_h;
    uint8_t version_l;
    uint8_t bms_status;                // 0x00 : wakeup, 0X01 :  discharge, 0x02 : charge, 0x03 : charger, 0x04 : precharge, 0x05 : charge_err, 0x06 : waterfall_light, 0x07 : self_discharge, 0x08 : junk.
    uint8_t SOC;                       // SOC 0-100%
    int32_t current;                   // （unit: mA)
    uint16_t cycle;                    // The current number of cycles of the battery
    std::array<int8_t, 2> BQ_NTC;      // x1 degrees centigrade
    std::array<int8_t, 2> MCU_NTC;     // x1 degrees centigrade
    std::array<uint16_t, 10> cell_vol; // cell voltage mV
  } BmsState;

  typedef struct
  {
    float x;
    float y;
    float z;
  } Cartesian;

  typedef struct
  {
    std::array<float, 4> quaternion;    // quaternion, normalized, (w,x,y,z)
    std::array<float, 3> gyroscope;     // angular velocity （unit: rad/s)    (raw data)
    std::array<float, 3> accelerometer; // acceleration （unit: m/(s2))       (raw data)
    std::array<float, 3> rpy;           // euler angle（unit: rad)
    int8_t temperature;                 // the temperature of imu (unit: °C)
  } IMU;                                // when under accelerated motion, the attitude of the robot calculated by IMU will drift.

  typedef struct
  {
    uint8_t r;
    uint8_t g;
    uint8_t b;
  } LED; // reserve

  typedef struct
  {
    uint8_t mode;       // motor working mode. Servo : 0x0A, Damping : 0x00，Overheat ： 0x08.
    float q;            // current angle (unit: radian)
    float dq;           // current velocity (unit: radian/second)
    float ddq;          // current acc (unit: radian/second*second)
    float tauEst;       // current estimated output torque (unit: N.m)
    float q_raw;        // reserve
    float dq_raw;       // reserve
    float ddq_raw;      // reserve
    int8_t temperature; // current temperature (temperature conduction is slow that leads to lag)
    std::array<uint32_t, 2> reserve;
  } MotorState; // motor feedback

  typedef struct
  {
    uint8_t mode; // desired working mode. Servo : 0x0A, Damping : 0x00.
    float q;      // desired angle (unit: radian)
    float dq;     // desired velocity (unit: radian/second)
    float tau;    // desired output torque (unit: N.m)
    float Kp;     // desired position stiffness (unit: N.m/rad )
    float Kd;     // desired velocity stiffness (unit: N.m/(rad/s) )
    std::array<uint32_t, 3> reserve;
  } MotorCmd; // motor control

  typedef struct
  {
    std::array<uint8_t, 2> head; // reserve
    uint8_t levelFlag;           // reserve
    uint8_t frameReserve;        // reserve

    std::array<uint32_t, 2> SN;      // reserve
    std::array<uint32_t, 2> version; // reserve
    uint16_t bandWidth;              // reserve
    IMU imu;
    std::array<MotorState, 20> motorState;
    BmsState bms;
    std::array<int16_t, 4> footForce;       // Data from foot airbag sensor
    std::array<int16_t, 4> footForceEst;    // reserve，typically zero
    uint32_t tick;                          // reference real-time from motion controller (unit: ms)
    std::array<uint8_t, 40> wirelessRemote; // Data from Unitree Joystick.
    uint32_t reserve;

    uint32_t crc;
  } LowState; // low level feedback

  typedef struct
  {
    std::array<uint8_t, 2> head; // reserve
    uint8_t levelFlag;           // reserve
    uint8_t frameReserve;        // reserve

    std::array<uint32_t, 2> SN;      // reserve
    std::array<uint32_t, 2> version; // reserve
    uint16_t bandWidth;
    std::array<MotorCmd, 20> motorCmd;
    BmsCmd bms;
    std::array<uint8_t, 40> wirelessRemote; // reserve
    uint32_t reserve;

    uint32_t crc;
  } LowCmd; // low level control

  typedef struct
  {
    std::array<uint8_t, 2> head; // reserve
    uint8_t levelFlag;           // reserve
    uint8_t frameReserve;        // reserve

    std::array<uint32_t, 2> SN;      // reserve
    std::array<uint32_t, 2> version; // reserve
    uint16_t bandWidth;
    IMU imu;
    std::array<MotorState, 20> motorState;
    BmsState bms;
    std::array<int16_t, 4> footForce;           // Data from foot airbag sensor
    std::array<int16_t, 4> footForceEst;        // reserve，typically zero
    uint8_t mode;                               // The current mode of the robot
    float progress;                             // reserve
    uint8_t gaitType;                           // 0.idle  1.trot  2.trot running  3.climb stair  4.trot obstacle
    float footRaiseHeight;                      // (unit: m, default: 0.08m), foot up height while walking
    std::array<float, 3> position;              // (unit: m), from own odometry in inertial frame, usually drift
    float bodyHeight;                           // (unit: m, default: 0.28m),
    std::array<float, 3> velocity;              // (unit: m/s), forwardSpeed, sideSpeed, rotateSpeed in body frame
    float yawSpeed;                             // (unit: rad/s), rotateSpeed in body frame
    std::array<float, 4> rangeObstacle;         // Distance to nearest obstacle
    std::array<Cartesian, 4> footPosition2Body; // foot position relative to body
    std::array<Cartesian, 4> footSpeed2Body;    // foot speed relative to body
    std::array<uint8_t, 40> wirelessRemote;     // Data from Unitree Joystick.
    uint32_t reserve;

    uint32_t crc;
  } HighState; // high level feedback

  typedef struct
  {
    std::array<uint8_t, 2> head; // reserve, no need to set.
    uint8_t levelFlag;           // reserve. No need to set, only need to set UDP class.
    uint8_t frameReserve;        // reserve

    std::array<uint32_t, 2> SN;      // reserve
    std::array<uint32_t, 2> version; // reserve
    uint16_t bandWidth;              // reserve
    uint8_t mode;                    // 0. idle, default stand
                                     // 1. force stand (controlled by dBodyHeight + ypr)
                                     // 2. target velocity walking (controlled by velocity + yawSpeed)
                                     // 3. target position walking (controlled by position + ypr[0]), reserve
                                     // 4. path mode walking (reserve for future release), reserve
                                     // 5. position stand down.
                                     // 6. position stand up
                                     // 7. damping mode
                                     // 8. recovery stand
                                     // 9. backflip, reserve
                                     // 10. jumpYaw, only left direction. Note, to use this mode, you need to set mode = 1 first.
                                     // 11. straightHand. Note, to use this mode, you need to set mode = 1 first.

    uint8_t gaitType;              // 0.idle
                                   // 1.trot
                                   // 2.trot running
                                   // 3.climb stair
                                   // 4.trot obstacle
    uint8_t speedLevel;            // reserve
    float footRaiseHeight;         // (unit: m, range: -0.06~0.03m, default: 0.09m), foot up height while walking, delta value
    float bodyHeight;              // (unit: m, range: -0.13~0.03m, default: 0.31m), delta value
    std::array<float, 2> position; // (unit: m), desired position in inertial frame, reserve
    std::array<float, 3> euler;    // (unit: rad), roll pitch yaw in stand mode
                                   // (range: roll : -0.75~0.75rad)
                                   // (range: pitch: -0.75~0.75rad)
                                   // (range: yaw  : -0.6~0.6rad)
    std::array<float, 2> velocity; // (unit: m/s), forwardSpeed, sideSpeed in body frame
                                   // (range: trot : vx:-1.1~1.5m/s,  vy:-1.0~1.0m/s)
                                   // (range: run  : vx:-2.5~3.5m/s,  vy:-1.0~1.0m/s)
                                   // (range: stair: vx:-0.2~0.25m/s, vy:-0.15~0.15m/s)
    float yawSpeed;                // (unit: rad/s), rotateSpeed in body frame
                                   // (range: trot : -4.0~4.0rad/s)
                                   // (range: run  : -4.0~4.0rad/s)
                                   // (range: stair: -0.7~0.7rad/s)
    BmsCmd bms;
    std::array<LED, 4> led;                 // reserve
    std::array<uint8_t, 40> wirelessRemote; // reserve
    uint32_t reserve;

    uint32_t crc;
  } HighCmd; // high level control

#pragma pack()

  typedef struct
  {
    unsigned long long TotalCount;    // total loop count
    unsigned long long SendCount;     // total send count
    unsigned long long RecvCount;     // total receive count
    unsigned long long SendError;     // total send error
    unsigned long long FlagError;     // total flag error
    unsigned long long RecvCRCError;  // total reveive CRC error
    unsigned long long RecvLoseError; // total lose package count
  } UDPState;                         // UDP communication state

} // namespace UNITREE_LEGGED_SDK

#endif
