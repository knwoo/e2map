#ifndef MOTOR_CTRL
#define MOTOR_CTRL

#ifdef __cplusplus
extern "C"{
#endif

#include "motor_msg.h" 
#include <stdint.h>

typedef struct {
    MasterComdDataV3  motor_send_data;  
	int hex_len;             
    long long send_time;           
    unsigned short id;  
    unsigned short mode;            
    //K_P*delta_Pos + K_W*delta_W + T
    float T;                      
    float W;                      
    float Pos;         
    float K_P;    
    float K_W;    
}MOTOR_send;

typedef struct
{
    ServoComdDataV3 motor_recv_data;    
    int hex_len;                  
    long long resv_time;           
    int correct;                   

    unsigned char motor_id;   
    unsigned char mode;        
    int Temp;               
    unsigned char MError;    

    float T;                       
    float W;                        
    float LW;                       
    int Acc;                    
    float Pos;                   

    float gyro[3];                  
    float acc[3];

}MOTOR_recv;

extern long long getSystemTime();       
extern int modify_data(MOTOR_send*);    
extern int extract_data(MOTOR_recv*);  
uint32_t crc32_core(uint32_t*, uint32_t);    

#ifdef __cplusplus
}
#endif

#endif