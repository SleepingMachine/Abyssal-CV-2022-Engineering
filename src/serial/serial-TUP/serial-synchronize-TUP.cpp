//
// Created by sleepingmachine on 2022/4/9.
//

#include "serial-port-TUP.hpp"
extern std::mutex mutex_serial_port_data;

void SerialPortTUP::SerialSynchronizeTUP(int64_t * sentData) {
    int mode,sentry,base;
    char ttyUSB_path[] = "/dev/ttyUSB1";//设置串口名称
    SerialPortTUP port(ttyUSB_path);//创建串口类对象
    port.initSerialPort();//串口初始化
    Mapdata data;
    int x,y,fall_flag;
    while (TRUE)
    {
        if (mutex_serial_port_data.try_lock()) {
            //y = *sentData % 1000;
            //x = (*sentData - y)/1000;

            fall_flag = *sentData % 10;
            y = (*sentData - fall_flag)/10 % 1000;
            x = ((*sentData - fall_flag)/10 - y)/1000;
            mutex_serial_port_data.unlock();
        }

        data = {6,fall_flag,x,y};

        port.TransformData(data);
        port.send();
        port.get_Mode(mode,sentry,base);
    }
}