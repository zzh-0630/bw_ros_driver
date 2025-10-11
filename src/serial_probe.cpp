// 此版本可用于测试串口通信，打印收到的所有帧

#include <fcntl.h>
#include <ros/ros.h>
#include <termios.h>
#include <unistd.h>

#include <cstdio>
#include <vector>

static uint8_t sum8(const uint8_t* p, size_t n) {
  unsigned s = 0;
  for (size_t i = 0; i < n; ++i) s += p[i];
  return s & 0xFF;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "bwsensing_serial_probe");
  ros::NodeHandle nh("~");
  std::string port = "/dev/ttyUSB0";
  int baud = 9600;
  nh.param("port", port, port);
  nh.param("baud", baud, baud);

  int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    perror("open");
    return 1;
  }
  termios tio{};
  tcgetattr(fd, &tio);
  cfmakeraw(&tio);
  speed_t sp = B9600;
  if (baud == 115200)
    sp = B115200;
  else if (baud == 57600)
    sp = B57600;
  else if (baud == 38400)
    sp = B38400;
  else if (baud == 19200)
    sp = B19200;
  cfsetispeed(&tio, sp);
  cfsetospeed(&tio, sp);
  tio.c_cflag |= (CLOCAL | CREAD | CS8);
  tio.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
  tio.c_iflag &= ~(IXON | IXOFF | IXANY);
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 2;  // 200ms
  tcsetattr(fd, TCSANOW, &tio);
  tcflush(fd, TCIOFLUSH);
  ROS_INFO("Probe opened %s @ %d", port.c_str(), baud);

  ros::Rate hz(100);
  std::vector<uint8_t> f = {
      0x77, 0x04, 0x00, 0x04,
      (uint8_t)((0x04 + 0x00 + 0x04) & 0xFF)};  // 读三轴角
  std::vector<uint8_t> buf;
  buf.reserve(256);
  enum { WAIT_77, READ_LEN, READ_PAYLOAD } st = WAIT_77;
  uint8_t need = 0;

  while (ros::ok()) {
    // 发一帧轮询
    (void)write(fd, f.data(), f.size());

    // 读并打印所有完整帧
    uint8_t tmp[256];
    ssize_t n = read(fd, tmp, sizeof(tmp));
    if (n > 0) {
      for (ssize_t i = 0; i < n; ++i) {
        uint8_t b = tmp[i];
        if (st == WAIT_77) {
          if (b == 0x77) {
            buf.clear();
            buf.push_back(b);
            st = READ_LEN;
          }
        } else if (st == READ_LEN) {
          buf.push_back(b);
          need = b;
          st = READ_PAYLOAD;
        } else {
          buf.push_back(b);
          if (buf.size() == (size_t)(1 + need)) {
            const uint8_t* p_len = &buf[1];
            uint8_t cs = sum8(p_len, need - 1);  // 只对“长度..数据域”求和
            uint8_t chk = buf[1 + need - 1];     // 校验和字节的位置

            // 打印时也只打印一整帧（不把下一帧起始77带上）
            printf("[frame] ");
            for (size_t k = 0; k < (size_t)(1 + need); ++k) {
              printf("%02X ", buf[k]);
            }
            printf(" | %s\n", (cs == chk) ? "OK" : "BAD");

            st = WAIT_77;
          }
        }
      }
    }
    ros::spinOnce();
    hz.sleep();
  }
  close(fd);
  return 0;
}
