/*!
*  \file         serial_port.hpp
*  \author       BW 
*  \date         15/10/2025
*  \brief        Serial port configuration structure.
*
*  Structure to store serial port configuration.
*
*  \section CodeCopyright Copyright Notice
*  MIT License
*
*  Copyright (C) 2025, BEWIS SENSING. All rights reserved.
*
*  Permission is hereby granted, free of charge, to any person obtaining a copy
*  of this software and associated documentation files (the "Software"), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
*  furnished to do so, subject to the following conditions:
*
*  The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/
#pragma once
#include <cstdint>
#include <string>
#include <termios.h>

namespace bw
{
/*!
 * 串口管理类
 */
class SerialPort
{
private:

  std::string port_;
  int baud_;
  int fd_;

  //---------------------------------------------------------------------//
  //- 私函数                                                             -//
  //---------------------------------------------------------------------//

  /*!
   * 串口配置
   */
  bool configureTermios();

  /*!
   * 波特率映射
   *
   * \param[in] baud 波特率值
   * \return 对应的 speed_t 值
   */
  static speed_t mapBaud(int baud);


public:

  //---------------------------------------------------------------------//
  //- 构造函数                                                          -//
  //--------------------------------------------------------------------//

  /*!
    * 默认构造函数。
    *
    * \param[in] port 串口设备路径（例如 "/dev/ttyUSB0"）。
    * \param[in] baud 波特率（例如 9600，115200）。
    */
  explicit SerialPort(std::string port = "/dev/ttyUSB0", int baud = 9600);

  /*!
   * 析构函数。
   */
  ~SerialPort();

  //---------------------------------------------------------------------//
  //- 函数                                                              -//
  //--------------------------------------------------------------------//

  /*!
   * 打开串口。
   */
  bool openSerial();

  /*!
   * 关闭串口。
   */
  void closeSerial();

  /*!
   * 重新打开串口。
   */
  bool reOpenSerial();

  /*!
   * 检查串口是否已打开。
   */
  bool isOpen() const { return fd_ >= 0; };

  /*!
   * 获取串口文件描述符。
   *
   * \return 文件描述符值，如果未打开则为 -1。
   */
  int fd() const { return fd_; };

  /*!
   * 设置串口波特率。
   *
   * \param[in] baud 波特率值。
   * \return 设置成功返回 true，否则返回 false。
   */
  bool setBaud(int baud);

  /*!
   * 获取当前串口波特率。
   */
  int getBaud() const { return baud_; };

  /*!
   * 设置串口设备路径。
   */
  void setPort(const std::string& p) { port_ = p; }

  /*!
   * 获取串口设备路径。
   */
  const std::string& getPort() const { return port_; }

  /*!
   * 串口写入数据。
   *
   * \param[in] data 数据缓冲区指针。
   * \param[in] len  数据长度。
   * \return 写入的字节数，或在出错时返回 -1。
   */
  ssize_t writeAll(const uint8_t* data, size_t len);

  /*!
   * 串口读取数据。
   *
   * \param[out] buf 数据缓冲区指针。
   * \param[in] max  最大读取字节数。
   * \return 读取的字节数，或在出错时返回 -1。
   */
  ssize_t readSome(uint8_t* buf, size_t max);

};

}
