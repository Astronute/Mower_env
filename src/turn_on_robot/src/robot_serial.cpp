#include "robot_serial.h"



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