#include "bw_ros_driver/serial_port.hpp"  

#include <cerrno>
#include <cstdio>
#include <cstring>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

using bw::SerialPort;

/*!
 * 串口管理类
 */

SerialPort::SerialPort(std::string port, int baud):
  port_(std::move(port)), baud_(baud), fd_(-1) {}

SerialPort::~SerialPort() 
{
  closeSerial();
}

speed_t SerialPort::mapBaud(int baud)
{
  switch (baud)
  {
    case 115200:
      return B115200;
    case 57600:
      return B57600;
    case 38400:
      return B38400;
    case 19200:
      return B19200;
    case 9600:
      return B9600;
    default:
      return B9600;
  }
}

bool SerialPort::configureTermios()
{
  termios tio{};
  if (tcgetattr(fd_, &tio) != 0) {
    std::perror("tcgetattr");
    return false;
  }

  cfmakeraw(&tio);
  speed_t sp = mapBaud(baud_);
  cfsetispeed(&tio, sp);
  cfsetospeed(&tio, sp);

  tio.c_cflag |= (CLOCAL | CREAD | CS8);
  tio.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);  // 8N1
  tio.c_iflag &= ~(IXON | IXOFF | IXANY);       // no flow control
  tio.c_cc[VMIN] = 0;                           // read non-blocking
  tio.c_cc[VTIME] = 2;                          // 200ms timeout

  if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
    std::perror("tcsetattr");
    return false;
  }
  tcflush(fd_, TCIOFLUSH);
  return true;
}

bool SerialPort::openSerial()
{
  closeSerial();
  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0)
  {
    perror("open");
    return false;
  }
  if (!configureTermios())
  {
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  return true;
}

void SerialPort::closeSerial() 
{
  if (fd_ >= 0) 
  {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::reOpenSerial() 
{
  closeSerial();
  return openSerial();
}

bool SerialPort::setBaud(int baud) 
{
  baud_ = baud;
  if (fd_ < 0) 
  {
    return true; 
  }
  return configureTermios();
}

ssize_t SerialPort::readSome(uint8_t* buf, size_t max)
{
  if (fd_ < 0)
  {
    return -1;
  }
  ssize_t n = ::read(fd_, buf, max);
  if (n < 0 && errno == EAGAIN)
  {
    return 0;  
  }
  return n;
}

ssize_t SerialPort::writeAll(const uint8_t* data, size_t len)
{
  if (fd_ < 0)
  {
    return -1;
  }
  size_t written = 0;
  while (written < len)
  {
    ssize_t n = ::write(fd_, data + written, len - written);
    if (n < 0)
    {
      if (errno == EAGAIN)
      {
        continue;
      }
      return -1;
    }
    written += static_cast<size_t>(n);
  }
  return static_cast<ssize_t>(written);
}