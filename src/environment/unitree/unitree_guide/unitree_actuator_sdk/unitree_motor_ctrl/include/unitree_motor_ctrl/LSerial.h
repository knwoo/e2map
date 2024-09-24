#ifndef LSERIAL
#define LSERIAL

#ifdef __cplusplus
extern "C"{
#endif

#include "motor_ctrl.h"

#if defined(__linux__)
    extern int open_set(char*);             
    extern int close_serial(int);           
    extern int broadcast(int, MOTOR_send*); 
    extern int send_recv(int, MOTOR_send*, MOTOR_recv*);    
                                                                   
#elif defined(__WIN32__)
    #include <windows.h>
    extern HANDLE open_set(char*);            
    extern int close_serial(HANDLE);
    extern int send_recv(HANDLE, MOTOR_send*, MOTOR_recv*);   
#endif

#ifdef __cplusplus
}
#endif

#endif