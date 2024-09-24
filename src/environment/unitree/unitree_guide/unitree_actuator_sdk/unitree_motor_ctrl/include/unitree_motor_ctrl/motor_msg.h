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

// typedef struct {
	
//   MasterComdData M1;
// 	MasterComdData M2;
// 	MasterComdData M3;
	
// }DMA485TxDataV3;

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

// typedef struct {
	
//   ServoComdDataV3 M[3];
//  // uint8_t  nullbyte1;
	
// }DMA485RxDataV3;


#pragma pack()

//  00 00 00 00 00 
//  00 00 00 00 00 
//  00 00 00 00 00 
//  00 00 00
/*
                 Tx485Data[_FR][i].head.start[0] = 0xFE ;     Tx485Data[_FR][i].head.start[1] = 0xEE; 					 
				 Tx485Data[_FR][i].Mdata.ModifyBit = 0xFF;    Tx485Data[_FR][i].Mdata.mode = 0;   				
				 Tx485Data[_FR][i].head.motorID = i;    0                                          
				 Tx485Data[_FR][i].Mdata.T = 0.0f;                         motor1.Extra_Torque = motorRxData[1].Mdata.T*0.390625f;    
				 Tx485Data[_FR][i].Mdata.Pos = 0x7FE95C80;                 
				 Tx485Data[_FR][i].Mdata.W = 16000.0f;                     motor1.Target_Speed =  motorRxData[1].Mdata.W*0.0078125f;   // rad/s	       
				 Tx485Data[_FR][i].Mdata.K_P = (q15_t)(0.6f*(1<<11));      motor1.K_Pos = ((float)motorRxData[1].Mdata.K_P)/(1<<11);   
				 Tx485Data[_FR][i].Mdata.K_W = (q15_t)(1.0f*(1<<10));      motor1.K_Speed = ((float)motorRxData[1].Mdata.K_W)/(1<<10);   
*/ 

#ifdef __cplusplus
}
#endif

#endif