//
// Created by sleepingmachine on 22-7-22.
//

#include "../include/serial/serial-port.hpp"
//extern std::mutex mutex2;
extern std::mutex mutex_serial_port_data_ore;
extern std::mutex mutex_serial_port_data_box;

extern std::atomic_bool serial_port_start;

extern std::atomic_bool configuration_file_read_complete;

static SerialConfig serialConfig = SerialConfigFactory::getSerialConfig();
SerialPort::SerialPort() {}

std::string SerialPort::read_device_;
std::string SerialPort::write_device_;
int SerialPort::baud_write_;
int SerialPort::baud_read_;

char SerialPort::sent_data_[8];
char SerialPort::read_data_[3];
char SerialPort::cache_read_data_[6] = {};

void SerialPort::SendData(int64* sent_data_ore, int64* sent_data_box) {
    serial_port_start = true;
    while (true) {
        while(!configuration_file_read_complete){
            cv::waitKey(10);
        }
        int temp_send_data_ore;
        int temp_send_data_box;

        if (mutex_serial_port_data_ore.try_lock()) {
            temp_send_data_ore = *sent_data_ore;
            mutex_serial_port_data_ore.unlock();
        }
        if (mutex_serial_port_data_box.try_lock()) {
            temp_send_data_box = *sent_data_box;
            mutex_serial_port_data_box.unlock();
        }
        //std::cout << temp_send_data_ore << " " << temp_send_data_box << std::endl;
        SerialPort::GetHitPointData(temp_send_data_ore, temp_send_data_box);

        int fd; /*File Descriptor*/
/*
        printf("\n +----------------------------------+");
        printf("\n |        Serial Port Write         |");
        printf("\n +----------------------------------+");
*/
        SerialPort::GetSerialInfo();
        SerialPort::CheckPortAvailability();

        fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
/*
        if (fd == -1) {
            printf("\n  ERROR ! in Opening ttyUSB0  ");
            //exit(0);

        }
        else{
            printf("\n  ttyUSB0 Opened Successfully ");
        }
*/
        struct termios SerialPortSettings;

        tcgetattr(fd, &SerialPortSettings);

        //设置波特率
        cfsetispeed(&SerialPortSettings, B115200);
        cfsetospeed(&SerialPortSettings, B115200);

        //设置没有校验
        SerialPortSettings.c_cflag &= ~PARENB;

        //停止位 = 1
        SerialPortSettings.c_cflag &= ~CSTOPB;
        SerialPortSettings.c_cflag &= ~CSIZE;

        //设置数据位 = 8
        SerialPortSettings.c_cflag |= CS8;

        SerialPortSettings.c_cflag &= ~CRTSCTS;
        SerialPortSettings.c_cflag |= CREAD | CLOCAL;

        //关闭软件流动控制
        SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);

        //设置操作模式
        SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        SerialPortSettings.c_oflag &= ~OPOST;

        SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        SerialPortSettings.c_oflag &= ~OPOST;
/*
        if ((tcsetattr(fd, TCSANOW, &SerialPortSettings)) != 0)
            printf("\n  ERROR ! in Setting attributes");
        else
            printf("\n  BaudRate = 115200 \n  StopBits = 1 \n  Parity   = none\n  ");
*/
        //定义传输内容
        char write_buffer[] = "Hello World";
        //传输字节数
        int bytes_written = 0;
        int bytes_readten = 0;
        //串口写数据
        bytes_readten = read (fd, SerialPort::read_data_, sizeof(read_data_));
        bytes_written = write(fd, SerialPort::sent_data_, sizeof(sent_data_));

        //std::cout << "接收了[" << bytes_readten << "]bytes的数据" << std::endl;

        int _mode_flag = -1;
        if (bytes_readten > 0){
            if (read_data_[0] == 's' && read_data_[2] == 'e'){
                //std::cout << "yesss" << std::endl;
                if (read_data_[1] == '0'){
                    _mode_flag = 0;
                }
                if (read_data_[1] == '1'){
                    _mode_flag = 1;
                }
            }
            else if(sizeof(cache_read_data_) == 0){
                strcat(cache_read_data_, read_data_);
            }
            else if(sizeof(cache_read_data_) > 0){
                strcat(cache_read_data_, read_data_);
                for (int i = 0; i < sizeof(cache_read_data_); ++i) {
                    if (cache_read_data_[i] == 's'){
                        //std::cout << "yesss" << std::endl;
                        if (read_data_[1] == '0'){
                            _mode_flag = 0;
                        }
                        if (read_data_[1] == '1'){
                            _mode_flag = 1;
                        }
                        memset(cache_read_data_,'\0',sizeof(cache_read_data_));

                        break;

                    }
                }
            }
        }
        /*
        if (_mode_flag == 0){
            DepthSolution::functionConfig_._mining_mode = MiningMode::GRIP_MODE;
        }
        else if(_mode_flag == 1){
            DepthSolution::functionConfig_._mining_mode = MiningMode::EXCHANGE_MODE;
        }
        */
        /*
        printf("\n  %d written to ttyUSB0", temp_send_data);
        printf("\n  %d Bytes written to ttyUSB0", bytes_written);
        printf("\n +----------------------------------+\n");
*/
        close(fd);
        //usleep(5000);
    }

}

void SerialPort::GetHitPointData(int64 temp_data_ore, int64 temp_data_box) {
/*
    SerialPort::sent_data_[0] = hitPointData / 100;
    SerialPort::sent_data_[1] = (hitPointData - SerialPort::sent_data_[0]*100) /10;
    SerialPort::sent_data_[2] = hitPointData % 10;
*/
    int ore_data_y = temp_data_ore % 1000;
    int ore_data_x = (temp_data_ore - ore_data_y) /1000;

    int box_data_y = temp_data_box % 1000;
    int box_data_x = (temp_data_box - box_data_y) /1000;

    //std::cout << ore_data_x << " " << ore_data_y << " " << box_data_x << " " << box_data_y << std::endl;

    sent_data_[0] = 's';

    sent_data_[1] = (( ore_data_x>> 8) & 0xFF);
    sent_data_[2] = (( ore_data_y>> 0) & 0xFF);
    sent_data_[3] = (( box_data_x>> 8) & 0xFF);
    sent_data_[4] = (( box_data_y>> 0) & 0xFF);

    sent_data_[5] = 'e';

    //std::cout << hitPointData_x << std::endl;
    //std::cout << hitPointData_y << std::endl;
    //std::cout << sent_data_[0] << " " << sent_data_[1] << std::endl;
    //sendData();
}

void SerialPort::CheckPortAvailability()
{
    /*
    std::cout << "  目前设定的读出串口 : " << read_device_ << "\n";
    std::cout << "  目前设定的写入串口 : " << write_device_ << "\n";
    */
    int fd   = open(read_device_.c_str(), O_EXCL, NULL);
    bool ret = false;
    if(fd < 0)
    {
        //std::cout << "读出串口 [" + read_device_ + "] 无响应" << "\n";
        ret = true;
    }
    close(fd);

    fd = open(write_device_.c_str(), O_EXCL, NULL);
    if(fd < 0)
    {
        //std::cout << "写入串口 [" + write_device_ + "] 无响应" << "\n";
        ret = true;
    }
    close(fd);

    if(ret)
    {
        //std::cout << "当前在线的串口 : " << "\n";
        //system("ls -l /dev/ttyUSB*");
        //exit(0);
    }
}

void SerialPort::GetSerialInfo() {
    read_device_ = serialConfig.readPortPath;
    write_device_ = serialConfig.writePortPath;
    baud_read_ = serialConfig.baud_read_Port;
    baud_write_ = serialConfig.baud_write_Port;
    //std::cout << "\n  getinfo" << " " << "baud set:" << baud_read_ << std::endl;
}

