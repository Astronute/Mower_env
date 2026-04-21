#include "robot_serial.h"



/*底盘反馈数据缓冲区*/
carMoveInfo_t g_tCarMoveInfo;
carMotorInfo_t g_tCarMotorInfo;
carBatteryInfo_t g_tCarBatteryInfo;
carImuAttitude_t g_tCarImuAttitudeInfo;
carImuRaw_t g_tCarImuRawInfo;
carTypeInfo_t g_tCarTypeInfo;

/*根据串口对应的设备，进行修改*/
const char default_path[] = "/dev/ttyUSB0";


//异或校验
uint8_t xor_check(uint8_t *_buf, uint8_t len)
{
    uint8_t xorTemp = _buf[0];

    for (int i = 1; i < len; i++)
    {
        xorTemp ^= (*(_buf + i));
    }
    return xorTemp;
}

/*根据缓存的数据，分别存入各个缓冲区*/
uint8_t carInfoParse(uint8_t *_buf, uint8_t _len)
{
    //根据异或校验判断数据是否存在接收错误
    if (_buf[_len - 1] == xor_check(&_buf[2], _len - 3))
    {

        switch (_buf[2])
        {
        case 0x02: // 速度、转向

            g_tCarMoveInfo.x_dir = _buf[3];
            g_tCarMoveInfo.x_lineSpeed = _buf[4] << 8 | _buf[5];
            g_tCarMoveInfo.y_dir = _buf[6];
            g_tCarMoveInfo.y_lineSpeed = _buf[7] << 8 | _buf[8];
            g_tCarMoveInfo.steerDir = _buf[9];
            g_tCarMoveInfo.steerAngle = _buf[10] << 8 | _buf[11];

            break;

        case 0x03: //电机转速
            g_tCarMotorInfo.motor1Speed = _buf[3] << 8 | _buf[4];
            g_tCarMotorInfo.motor2Speed = _buf[5] << 8 | _buf[6];
            g_tCarMotorInfo.motor3Speed = _buf[7] << 8 | _buf[8];
            g_tCarMotorInfo.motor4Speed = _buf[9] << 8 | _buf[10];

            break;

        case 0x04: //电压
            g_tCarBatteryInfo.voltage = _buf[3];
            //printf("%x\n", g_tCarBatteryInfo.voltage );
            break;

        case 0x05: //IMU pitch roll yaw
            g_tCarImuAttitudeInfo.pitchSymbol = _buf[3];
            g_tCarImuAttitudeInfo.pitch = _buf[4] << 8 | _buf[5];
            g_tCarImuAttitudeInfo.rollSymbol = _buf[6];
            g_tCarImuAttitudeInfo.roll = _buf[7] << 8 | _buf[8];
            g_tCarImuAttitudeInfo.yawSymbol = _buf[9];
            g_tCarImuAttitudeInfo.yaw = _buf[10] << 8 | _buf[11];

        case 0x06: //imu 原始数据
            g_tCarImuRawInfo.gyroxSymbol = _buf[3];
            g_tCarImuRawInfo.gyrox = _buf[4] << 8 | _buf[5];
            g_tCarImuRawInfo.gyroySymbol = _buf[6];
            g_tCarImuRawInfo.gyroy = _buf[7] << 8 | _buf[8];
            g_tCarImuRawInfo.gyrozSymbol = _buf[9];
            g_tCarImuRawInfo.gyroz = _buf[10] << 8 | _buf[11];
            g_tCarImuRawInfo.accelxSymbol = _buf[12];
            g_tCarImuRawInfo.accelx = _buf[13] << 8 | _buf[14];
            g_tCarImuRawInfo.accelySymbol = _buf[15];
            g_tCarImuRawInfo.accely = _buf[16] << 8 | _buf[17];
            g_tCarImuRawInfo.accelzSymbol = _buf[18];
            g_tCarImuRawInfo.accelz = _buf[19] << 8 | _buf[20];
            g_tCarImuRawInfo.quatwSymbol = _buf[21];
            g_tCarImuRawInfo.quatw = _buf[22] << 8 | _buf[23];
            g_tCarImuRawInfo.quatxSymbol = _buf[24];
            g_tCarImuRawInfo.quatx = _buf[25] << 8 | _buf[26];
            g_tCarImuRawInfo.quatySymbol = _buf[27];
            g_tCarImuRawInfo.quaty = _buf[28] << 8 | _buf[29];
            g_tCarImuRawInfo.quatzSymbol = _buf[30];
            g_tCarImuRawInfo.quatz = _buf[31] << 8 | _buf[32];
            ;
            break;

        case 0x07: //车辆类型
            g_tCarTypeInfo.carType = _buf[3];

            break;

        default:

            break;
        }
    }
}

/*缓存每一帧数据，并缓存下来*/
void packet_unpack(uint8_t _buf)
{
    static uint8_t uart_flag = 1;
    static uint8_t s_uartBuf[100];
    static uint8_t s_len = 0;
    if (_buf == 0xFD)
    {
        s_uartBuf[0] = 0xFD;
        uart_flag = 1;
        s_len++;
    }
    else
    {
        if (uart_flag == 1)
        {
            if (s_len > s_uartBuf[1] + 1)
            {
                s_uartBuf[s_len] = _buf;
                s_len++;

                carInfoParse(s_uartBuf, s_len);
                uart_flag = 0;
                s_len = 0;
                return;
            }
            else
            {
                s_uartBuf[s_len] = _buf;
                s_len++;
            }
        }
    }
}

/*小车控制函数， 
    fd ： 串口设备的文件描述符
    _xLineSpeed: 小车x轴的线速度 (mm/s)
    _yLineSpeed：小车y轴的线速度
    _steerAngle：小车的转向速度 (*0.01rad/s)
*/
void sendCarControlCmd(int *fd, int16_t _xLineSpeed, int16_t _yLineSpeed, int16_t _steerAngle)
{
    uint8_t _buf[13] = {0};
    _buf[0] = 0xCD;
    _buf[1] = 0x0a;
    _buf[2] = 0x01;

    if (_xLineSpeed > 0)
    {
        _buf[3] = 0x00;
    }
    else
    {
        _buf[3] = 0x01;
    }

    _buf[4] = (abs(_xLineSpeed) & 0xff00) >> 8;
    _buf[5] = (abs(_xLineSpeed) & 0x00ff);

    if (_yLineSpeed > 0)
    {
        _buf[6] = 0x00;
    }
    else
    {
        _buf[6] = 0x01;
    }

    _buf[7] = (abs(_yLineSpeed) & 0xff00) >> 8;
    _buf[8] = (abs(_yLineSpeed) & 0x00ff);

    if (_steerAngle > 0)
    {
        _buf[9] = 0x00;
    }
    else
    {
        _buf[9] = 0x01;
    }

    _buf[10] = (abs(_steerAngle) & 0xff00) >> 8;
    _buf[11] = (abs(_steerAngle) & 0x00ff);

    _buf[12] = xor_check(&_buf[2], _buf[1]);

    write(*fd, _buf, 13);
}


// int main(int argc, char *argv[])
// {

//     int fd;
//     int res;
//     char *path;
//     char buf;

//     //若无输入参数则使用默认终端设备
//     if (argc > 1)
//         path = argv[1];
//     else
//         path = (char *)default_path;

//     //获取串口设备描述符
//     printf("This is tty/usart demo.\n");
//     fd = open(path, O_RDWR);
//     if (fd < 0)
//     {
//         printf("Fail to Open %s device\n", path);
//         return 0;
//     }

//     struct termios opt;

//     //清空串口接收缓冲区
//     tcflush(fd, TCIOFLUSH);
//     // 获取串口参数 opt
//     tcgetattr(fd, &opt);

//     //设置串口输出波特率
//     cfsetospeed(&opt, B115200);
//     //设置串口输入波特率
//     cfsetispeed(&opt, B115200);
//     //设置数据位数
//     opt.c_cflag &= ~CSIZE;
//     opt.c_cflag |= CS8;
//     //校验位
//     opt.c_cflag &= ~PARENB;
//     opt.c_iflag &= ~INPCK;
//     //设置停止位
//     opt.c_cflag &= ~CSTOPB;

//     //更新配置
//     tcsetattr(fd, TCSANOW, &opt);

//     opt.c_iflag &= ~(INLCR); /*禁止将输入中的换行符NL映射为回车-换行CR*/

//     opt.c_iflag &= ~(IXON | IXOFF | IXANY); //不要软件流控制
//     opt.c_oflag &= ~OPOST;
//     opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); //原始模式

//     tcsetattr(fd, TCSANOW, &opt); //更新终端配置

//     printf("device %s is set to 115200bps, 8N1\n", path);

//     while (1)
//     {
//         /*发送控制命令，速度100， 左转向10°*/
//         sendCarControlCmd(&fd, 150, 0, 0);
//         // 接收字符串
//         res = read(fd, &buf, 1);
//         if (res > 0)
//         {

//             /*数据解析*/
//             packet_unpack(buf);
//         }
//     }

//     printf("read error,res = %d", res);

//     close(fd);
//     return 0;
// }