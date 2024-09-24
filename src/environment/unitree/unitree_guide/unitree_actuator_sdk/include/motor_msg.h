#ifndef MOTOR_MSG
#define MOTOR_MSG

#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>
typedef int16_t q15_t;

#pragma pack(1)

typedef union{
        int32_t           L;
        uint8_t       u8[4];
       uint16_t      u16[2];
       uint32_t         u32;
          float           F;
}COMData32;

typedef struct {
    unsigned char  start[2];     
	unsigned char  motorID;     
	unsigned char  reserved;
}COMHead;

#pragma pack()

#pragma pack(1)

typedef struct { 
	
	   uint8_t  fan_d;      
	   uint8_t  Fmusic;      
	   uint8_t  Hmusic;      
	   uint8_t  reserved4;
	
	   uint8_t  FRGB[4];     
	
}LowHzMotorCmd;

typedef struct { 
    uint8_t  mode;     
    uint8_t  ModifyBit;   
    uint8_t  ReadBit;     
    uint8_t  reserved;

    COMData32  Modify;     
    //K_P*delta_Pos + K_W*delta_W + T
    q15_t     T;      
    q15_t     W;     
    int32_t   Pos;     

    q15_t    K_P;     
    q15_t    K_W;     

    uint8_t LowHzMotorCmdIndex;     
    uint8_t LowHzMotorCmdByte;  
	
     COMData32  Res[1];    
	
}MasterComdV3;  

typedef struct {	
    COMHead head;    
    MasterComdV3 Mdata;
    COMData32 CRCdata;
}MasterComdDataV3;

#pragma pack()

#pragma pack(1)

typedef struct {
    uint8_t  mode;       
    uint8_t  ReadBit;     
    int8_t  Temp;        
    uint8_t  MError;      
 
    COMData32  Read;  
    int16_t     T;      

    int16_t     W;     
    float      LW;   

    int16_t     W2;      
    float      LW2;   

    int16_t    Acc;           
    int16_t    OutAcc;        
		 
    int32_t   Pos;      
    int32_t   Pos2;

    int16_t     gyro[3];  
    int16_t     acc[3];   

    int16_t     Fgyro[3];  
    int16_t     Facc[3];
    int16_t     Fmag[3];
    uint8_t     Ftemp;     
    
    int16_t     Force16;
    int8_t      Force8; 
		
    uint8_t     FError;   
		
    int8_t      Res[1];    
	
}ServoComdV3;  

typedef struct {
    COMHead        head;
    ServoComdV3      Mdata;

    COMData32    CRCdata;

}ServoComdDataV3;	

#pragma pack()

#ifdef __cplusplus
}
#endif

#endif