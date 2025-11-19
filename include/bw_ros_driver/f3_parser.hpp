/*!
 *  \file         f3_parser.hpp
 *  \author       BW
 *  \date         2025-10-15
 *  \brief        F3 协议解析：0xF3 | CmdID | DataLen | Count | CRC8 | Data |
 * CheckSum
 */
#pragma once

#include <cstdint>
#include <vector>

namespace bw {

/*!
 * \brief F3 协议一帧解出的样本
 *
 * 单位说明：
 *  - pitch_deg / roll_deg / yaw_deg : 角度（度）
 *  - acc_*_g                        : 加速度（g）
 *  - gyro_*_dps                     : 角速度（度/秒）
 *  - mag_*                          : 磁场（高斯 G）
 *  - temperature_deg_c              : 温度（摄氏度）
 *  - time_us                        : 内部计数（微秒）
 */
struct F3Sample {
  bool valid = false;

  uint8_t cmd_id = 0;
  uint8_t count = 0;

  float pitch_deg = 0.0f;
  float roll_deg = 0.0f;
  float yaw_deg = 0.0f;

  float acc_x_g = 0.0f;
  float acc_y_g = 0.0f;
  float acc_z_g = 0.0f;

  float gyro_x_dps = 0.0f;
  float gyro_y_dps = 0.0f;
  float gyro_z_dps = 0.0f;

  float mag_x = 0.0f;
  float mag_y = 0.0f;
  float mag_z = 0.0f;

  float temperature_deg_c = 0.0f;
  uint32_t time_us = 0;
};

/*!
 * \brief 串行 F3 流解析器
 *
 * 状态机：
 *  - WAIT_HEAD：寻找 0xF3
 *  - READ_HEADER：收齐 [0]=Head, [1]=CmdID, [2]=DataLen
 *  - READ_FRAME：根据 DataLen 知道总长度，收齐整帧，做 CRC8 + CheckSum
 * 校验，再解析
 */
class F3Parser {
 public:
  F3Parser();

  //! 清空状态（计数器保留）
  void reset();

  /*!
   * \brief 喂入一批字节，可能产出 0..N 个样本
   *
   * \param[in]  data  输入字节流
   * \param[in]  n     字节数
   * \param[out] out   新产出的样本 push_back 到该 vector 中
   * \return     本次 feed 新产出的样本数量
   */
  size_t feed(const uint8_t* data, size_t n, std::vector<F3Sample>& out);

  //! 成功通过 CRC8 + CheckSum 的帧数
  uint64_t ok() const { return ok_cnt_; }

  //! 帧级错误数量（长度非法 / CheckSum 错误等）
  uint64_t bad() const { return bad_cnt_; }

  //! CRC8 校验错误数量（Head..Count 的 CRC8 不通过）
  uint64_t crcBad() const { return crc_bad_cnt_; }

 private:
  enum State {
    WAIT_HEAD,
    READ_HEADER,  //!< 收到 Head(0xF3)、CmdID、DataLen
    READ_FRAME    //!< 已知 DataLen，等待整帧收齐
  };

  State state_;
  std::vector<uint8_t> buf_;  //!< 当前在累积的帧
  size_t expected_len_;       //!< 本帧期望总长度（字节数）
  uint64_t ok_cnt_;
  uint64_t bad_cnt_;
  uint64_t crc_bad_cnt_;

  /*!
   * \brief 计算 CRC8 (Poly=0x07, Init=0x00)
   * \param[in] p   缓冲区
   * \param[in] len 字节数
   */
  static uint8_t computeCrc8(const uint8_t* p, size_t len);

  /*!
   * \brief 在已经通过 CRC8 和 CheckSum 的前提下，解析一帧
   */
  bool parseFrame(const std::vector<uint8_t>& frame, F3Sample& sample);
};

}  // namespace bw
