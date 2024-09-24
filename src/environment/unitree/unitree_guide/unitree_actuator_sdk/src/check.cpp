#include <stdio.h>
#include <errno.h>     
#include <string.h>
#include <unistd.h>    
#include <sys/time.h> 
#include "LSerial.h"    
#include "motor_ctrl.h"

int main()
{
    MOTOR_send motor_s, motor_s1;
    // long long start_time = 0;
    motor_s.id = 0;
    motor_s.mode = 5;
    motor_s.T = 0;            //Nm, T<255.9
    motor_s.W = 21.0;               //rad/s, W<511.9
    motor_s.Pos = 0.0;          //rad, Pos<131071.9
    motor_s.K_P = 0.0;        //K_P<31.9
    motor_s.K_W = 10;        //K_W<63.9

    motor_s1.id = 0;
    motor_s1.mode = 0;
    MOTOR_recv motor_r;
#if defined(__linux__)
    int fd;
    
#elif defined(__WIN32__)
    HANDLE fd;

#endif

    fd = open_set((char*)"/dev/ttyUSB0");
    // fd = open_set((char*)"\\\\.\\COM4");
    
    modify_data(&motor_s);
    modify_data(&motor_s1);

    int sta;
    sta = send_recv(fd, &motor_s1, &motor_r);
    // printf("status2: %d\n", sta);
    extract_data(&motor_r);
    // show_resv_data_hex(&motor_r);
    printf("START\n");
    // show_resv_data(&motor_r);


    for(int i=0; i<100; i++)
    {
        send_recv(fd, &motor_s, &motor_r);
        // extract_data(&motor_r);
        // show_resv_data(&motor_r);
        usleep(100000);
    }

    sta = send_recv(fd, &motor_s1, &motor_r);
    // printf("status2: %d\n", sta);
    extract_data(&motor_r);
    // show_resv_data_hex(&motor_r);
    printf("END\n");
    // show_resv_data(&motor_r);

    close_serial(fd);
#if defined(__WIN32__)
    system("pause");
#endif
    
    return 0;
}
