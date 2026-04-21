#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <string.h>
#include <sys/ioctl.h>
#include <stdint.h>


typedef struct
{
    uint8_t head;
    uint8_t frame_len;
    uint8_t frame_code;
    uint8_t *frame_data;
    uint8_t frame_crc;

} uart_frame_t;

typedef struct
{

    uint8_t x_dir;        //x轴方向
    uint16_t x_lineSpeed; //x轴线速度
    uint8_t y_dir;        //Y轴方向
    uint16_t y_lineSpeed; //y轴线速度
    uint8_t steerDir;     //转向方向
    uint16_t steerAngle;  //转向角度

} carMoveInfo_t;

typedef struct
{
    uint16_t motor1Speed; // 电机1转速
    uint16_t motor2Speed; // 电机2转速
    uint16_t motor3Speed; // 电机3转速
    uint16_t motor4Speed; // 电机4转速

} carMotorInfo_t;

typedef struct
{
    uint8_t voltage; //电压大小
} carBatteryInfo_t;

typedef struct
{

    uint8_t pitchSymbol; //picth 正负
    uint16_t pitch;
    uint8_t rollSymbol; //roll 正负
    uint16_t roll;
    uint8_t yawSymbol; //yaw  正负
    uint16_t yaw;

} carImuAttitude_t;

typedef struct
{
    uint8_t gyroxSymbol;
    uint8_t gyrox;

    uint8_t gyroySymbol;
    uint8_t gyroy;

    uint8_t gyrozSymbol;
    uint8_t gyroz;

    uint8_t accelxSymbol;
    uint8_t accelx;

    uint8_t accelySymbol;
    uint8_t accely;

    uint8_t accelzSymbol;
    uint8_t accelz;

    uint8_t quatwSymbol;
    uint8_t quatw;

    uint8_t quatxSymbol;
    uint8_t quatx;

    uint8_t quatySymbol;
    uint8_t quaty;

    uint8_t quatzSymbol;
    uint8_t quatz;

} carImuRaw_t;

typedef struct
{

    uint8_t carType;

} carTypeInfo_t;



void sendCarControlCmd(int *fd, int16_t _xLineSpeed, int16_t _yLineSpeed, int16_t _steerAngle);

/*串口解析*/
void packet_unpack(uint8_t buf);

//异或校验
uint8_t xor_check(uint8_t *_buf, uint8_t len);

/*根据缓存的数据，分别存入各个缓冲区*/
uint8_t carInfoParse(uint8_t *_buf, uint8_t _len);

void sendCarControlCmd(int *fd, int16_t _xLineSpeed, int16_t _yLineSpeed, int16_t _steerAngle);

