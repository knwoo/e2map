#ifndef LSERIAL
#define LSERIAL

#ifdef __cplusplus
extern "C"{
#endif

#include "motor_ctrl.h"

#if defined(__linux__)
    extern int open_set(char*);             //Input serial port name. (The Baud rate is fixed to 4800000), return the handle of serial port;
    extern int close_serial(int);           //Input serial port handle. Close the serial port
    extern int broadcast(int, MOTOR_send*); //Input serial port handle and the pointer of message to be sent. Broadcast to all motors, and the motors do not have any respond
    extern int send_recv(int, MOTOR_send*, MOTOR_recv*);    //Input serial port handle, the pointer of message to be sent and the pointer of received message.
                                                            //Synchronous send and receive, send 34 bytes and receive 78 bytes
                                                            //The return value is the status of send and receive. 0:send fail and receive fail, 1:send success and receive fail, 11:send success and receive success
#elif defined(__WIN32__)
    #include <windows.h>
    extern HANDLE open_set(char*);             //Input serial port name. (The Baud rate is fixed to 4800000), return the handle of serial port;
    extern int close_serial(HANDLE);           //Input serial port handle. Close the serial port
    extern int broadcast(HANDLE, MOTOR_send*); //Input serial port handle and the pointer of message to be sent. Broadcast to all motors, and the motors do not have any respond
    extern int send_recv(HANDLE, MOTOR_send*, MOTOR_recv*);    //Input serial port handle, the pointer of message to be sent and the pointer of received message.
                                                            //Synchronous send and receive, send 34 bytes and receive 78 bytes
                                                            //The return value is the status of send and receive. 0:send fail and receive fail, 1:send success and receive fail, 11:send success and receive success
#endif

#ifdef __cplusplus
}
#endif

#endif