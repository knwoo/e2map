#ifndef MOTOR_CTRL
#define MOTOR_CTRL

#ifdef __cplusplus
extern "C"{
#endif

#include "motor_msg.h"  //The data structure of motor communication messages
#include <stdint.h>

//Define the message to be send.
typedef struct {
    MasterComdDataV3  motor_send_data;  //The data to be sent to motor. Details are shown in motor_msg.h
	int hex_len;                    //The Bytes count of the message to be sent, it should be 34 for this motor
    long long send_time;            //The time that message was sent
    //The values of motor commands 
    unsigned short id;              //Motor ID
    unsigned short mode;            //The control mode, 0:free, 5:Open loop slow turning, 10:close loop control
    //The following parameters are just motor's parameters, do not concern the reducer. The real torque command to control board is:
    //K_P*delta_Pos + K_W*delta_W + T
    float T;                        //Desired output torque of motor【（Nm）】
    float W;                        //Desired output speed of motor【(rad/s)】
    float Pos;                      //Desired shaft position of motor【（rad）】
    float K_P;                      //The position stiffness
    float K_W;                      //The speed stiffness
}MOTOR_send;

//Define the data structure of received message. 
typedef struct
{
    ServoComdDataV3 motor_recv_data;     //The data received from motor. Details are shown in motor_msg.h
    int hex_len;                    //The Bytes count of the received message, it should be 78 for this motor
    long long resv_time;            //The time of receiving this message, microsecond(us)【接收该命令的时间, 微秒(us)】
    int correct;                    //Whether the received data is correct(1:correct, 0:wrong)
    unsigned char motor_id;         //Motor ID
    unsigned char mode;             //The control mode, 0:free, 5:Open loop slow turning, 10:close loop control
    int Temp;                       //Temperature
    unsigned char MError;           //Error code

    float T;                        //The output torque of motor
    float W;                        //The motor shaft speed(without filter)
    float LW;                       //The motor shaft speed(with filter)
    int Acc;                        //The acceleration of motor shaft
    float Pos;                      //The motor shaft position(control board zero fixed)

    float gyro[3];                  //The data of 6 axis inertial sensor on the control board
    float acc[3];

}MOTOR_recv;

extern long long getSystemTime();       //Get the current system time, microsecond(us)
extern int modify_data(MOTOR_send*);    //Compile the data to the data structure of motor
extern int extract_data(MOTOR_recv*);   //Extract the parameter values from received data
uint32_t crc32_core(uint32_t*, uint32_t);    //Calculate the CRC. Inputs are: pointer to the data to be calculated, Bytes count/4,(ignore remainder)

#ifdef __cplusplus
}
#endif

#endif