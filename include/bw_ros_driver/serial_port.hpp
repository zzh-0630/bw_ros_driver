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
*  Copyright (c) 2025 BW
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
 * Class to manage a serial port.
 */
class SerialPort
{
private:

  std::string port_;
  int baud_;
  int fd_;

  //---------------------------------------------------------------------//
  //- Private  methods                                                  -//
  //---------------------------------------------------------------------//

  /*!
   * Configure the termios structure.
   */
  bool configureTermios();

  /*!
   * Map an integer baud rate to a speed_t value.
   *
   * \param[in] baud Baud rate as an integer.
   * \return Corresponding speed_t value, or B0 if unsupported.
   */
  static speed_t mapBaud(int baud);


public:

  //---------------------------------------------------------------------//
  //- Constructor                                                       -//
  //---------------------------------------------------------------------//

  /*!
    * Default constructor.
    *
    * \param[in] port Serial port name (e.g., "/dev/ttyUSB0").
    * \param[in] baud Baud rate (e.g., 9600, 115200).
    */
  explicit SerialPort(std::string port = "/dev/ttyUSB0", int baud = 9600);

  /*!
   * Destructor.
   */
  ~SerialPort();

  //---------------------------------------------------------------------//
  //- Parameters                                                        -//
  //---------------------------------------------------------------------//

  //---------------------------------------------------------------------//
  //- Operations                                                        -//
  //---------------------------------------------------------------------//

  /*!
   * Open the serial port with the specified configuration.
   */
  bool openSerial();

  /*!
   * Close the serial port.
   */
  void closeSerial();

  /*!
   * Reopen the serial port.
   */
  bool reOpenSerial();

  /*!
   * Check if the serial port is open.
   */
  bool isOpen() const { return fd_ >= 0; };

  /*!
   * Get the file descriptor of the serial port.
   *
   * \return File descriptor, or -1 if not open.
   */
  int fd() const { return fd_; };

  /*!
   * Set the baud rate of the serial port.
   *
   * \param[in] baud Baud rate (e.g., 9600, 115200).
   * \return True if successful, False otherwise.
   */
  bool setBaud(int baud);

  /*!
   * Get the current baud rate of the serial port.
   */
  int getBaud() const { return baud_; };

  /*!
   * Set the current port name of the serial port.
   */
  void setPort(const std::string& p) { port_ = p; }

  /*!
   * Get the current port name of the serial port.
   */
  const std::string& getPort() const { return port_; }

  /*!
   * Write data to the serial port.
   *
   * \param[in] data Data buffer to write.
   * \param[in] len  Number of bytes to write.
   * \return Number of bytes written, or -1 on error.
   */
  ssize_t writeAll(const uint8_t* data, size_t len);

  /*!
   * Read data from the serial port.
   *
   * \param[out] buf Buffer to store read data.
   * \param[in] max  Maximum number of bytes to read.
   * \return Number of bytes read, or -1 on error.
   */
  ssize_t readSome(uint8_t* buf, size_t max);

};

}
