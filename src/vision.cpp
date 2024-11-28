/** 
 *******************************************************************************
 * @file      : vision.cpp
 * @brief     : 
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "vision.hpp"

#include <cmath>
#include <cstring>

#include "tick.hpp"
/* Private macro -------------------------------------------------------------*/
namespace robot
{
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

bool Vision::encode(size_t &len, uint8_t *data)
{
  // 增加编码失败计数
  encode_fail_cnt_++;

  // 检查数据长度是否足够
  if (len < kTxDataLen) {
    return false;
  }

  // 初始化数据缓冲区
  tx_data_buffer_[0] = 0x3f;  // 固定值，可能用于标识数据包的开始
  tx_data_buffer_[1] = 0x4f;  // 固定值，可能用于标识数据包的类型

  // 将工作状态编码到数据缓冲区
  tx_data_buffer_[2] = (uint8_t)tx_data_.work_state;

  // 常量定义
  const float angleFactor = 100.0f / M_PI * 180.0f;

  // 将目标速度编码到数据缓冲区
  int16_t temp = tx_data_.blt_spd * 100;
  tx_data_buffer_[3] = temp >> 8;  // 高字节
  tx_data_buffer_[4] = temp;       // 低字节

  // 将目标角度（roll）编码到数据缓冲区
  temp = tx_data_.roll * angleFactor;
  tx_data_buffer_[5] = temp >> 8;  // 高字节
  tx_data_buffer_[6] = temp;       // 低字节

  // 将目标角度（pitch）编码到数据缓冲区
  temp = tx_data_.pitch * angleFactor;
  tx_data_buffer_[7] = temp >> 8;  // 高字节
  tx_data_buffer_[8] = temp;       // 低字节

  // 将目标角度（yaw）编码到数据缓冲区
  temp = tx_data_.yaw * angleFactor;
  tx_data_buffer_[9] = temp >> 8;  // 高字节
  tx_data_buffer_[10] = temp;      // 低字节

  // 将目标颜色编码为一个字节的低2位
  tx_data_buffer_[11] = ((uint8_t)tx_data_.tgt_color) & 0b11;
  if (tx_data_.balance != Balance::kNoBalance) {
    tx_data_buffer_[11] |= 1u << ((uint8_t)tx_data_.balance - 1);
  }

  // 计算校验位，将数据缓冲区的第3个字节到第12个字节异或运算得到校验位
  tx_data_buffer_[12] = tx_data_buffer_[2];
  for (size_t i = 3; i < 12; i++) {
    tx_data_buffer_[12] ^= tx_data_buffer_[i];
  }

  // 将编码后的数据复制到输出缓冲区
  memcpy(data, tx_data_buffer_, kTxDataLen);

  // 更新编码成功计数
  encode_fail_cnt_--;
  encode_success_cnt_++;
  len = kTxDataLen;
  return true;
}

bool Vision::decode(size_t len, const uint8_t *data)
{
  n_rx_frames_ = 0;
  if (data == nullptr || len == 0) {
    decode_fail_cnt_++;
    return false;
  }
  for (size_t i = 0; i < len; i++) {
    rx_result_ = processByte(data[i]);
    if (!isProcessByteNoErr()) {
      decode_fail_cnt_++;
    } else if (isProcessByteOk()) {
      n_rx_frames_++;
    }
  }
  if (n_rx_frames_ != 0) {
    oc_.update();

    is_updated_ = true;
    if (update_cb_) {
      update_cb_();
    }
  }

  return n_rx_frames_ != 0;
};

void Vision::resetRxData() { rx_data_ = RxData(); };

void Vision::resetTxData()
{
  tx_data_ = TxData();
  tx_data_.blt_spd = default_blt_spd_;
};

/* Private function definitions ----------------------------------------------*/

/**
 * @brief 处理一个字节的数据。
 *
 * 该函数根据当前的解码状态 `rx_status_` 来解码输入的字节。解码的过程包括接收起始位，接收头部，以及接收固定长度的数据帧。
 * 数据帧接收完毕后，进行校验，并解析其中的数据信息，包含角度、目标ID、颜色等信息。
 * 在解码过程中，任何步骤出错（包括校验失败和数据帧格式不符合预期）都会重置解码进度，并返回 `kErrUndefined` 表示处理失败。
 * 如果字节成功解码，函数将返回 `kOk`。
 * 该函数只处理一个字节的数据，所以会被连续地调用来处理多个字节的数据。
 *
 * @param byte 需要处理的字节。
 * @return Vision::RxResult 返回处理的结果。如果成功，返回 `kOk`；否则返回相应的错误码。
 */
Vision::RxResult Vision::processByte(uint8_t byte)
{
  RxResult result = RxResult::kErrUndefined;
  if (rx_status_ == RxStatus::kWaitingHeaderSof) {
    // 等待起始位
    result = RxResult::kHandlingWaitSof;
    if (byte == 0xAA) {
      rx_data_buffer_[rx_data_buffer_idx_++] = 0xAA;
      rx_status_ = RxStatus::kWaitingHeaderCplt;
    } else {
      result = RxResult::kErrNoDataInput;
      resetDecodeProgress();
    }
  } else if (rx_status_ == RxStatus::kWaitingHeaderCplt) {
    // 接收帧头，加上 SOF 为 0xAA 0xBB 0xCC
    result = RxResult::kHandlingHeader;
    if (byte == 0xBB && rx_data_buffer_idx_ == 1) {
      rx_data_buffer_[rx_data_buffer_idx_++] = 0xBB;
      rx_status_ = RxStatus::kWaitingHeaderCplt;
    } else if (byte == 0xCC && rx_data_buffer_idx_ == 2) {
      rx_data_buffer_[rx_data_buffer_idx_++] = 0xCC;
      rx_status_ = RxStatus::kWaitingTailCplt;
    } else {
      result = RxResult::kErrFailedHeader;
      resetDecodeProgress();
    }
  } else if (rx_status_ == RxStatus::kWaitingTailCplt) {
    // 接收固定长度的数据
    do {
      result = RxResult::kHandlingWaitTail;
      if (rx_data_buffer_idx_ < kRxDataLen) {
        rx_data_buffer_[rx_data_buffer_idx_++] = byte;
      }
      if (rx_data_buffer_idx_ != kRxDataLen) {
        break;
      }
      // 接收完所有数据，开始校验
      // 帧尾固定数据位：0xFF
      if (rx_data_buffer_[kRxDataLen - 1] != 0xFF) {
        result = RxResult::kErrFailedTail;
        resetDecodeProgress();
        break;
      }
      // 校验位：rx_data_buffer_[9]
      uint8_t checksum = rx_data_buffer_[3];
      uint32_t checksum_idx = kRxDataLen - 2;
      for (size_t i = 4; i < checksum_idx; i++) {
        checksum ^= rx_data_buffer_[i];
      }
      if (checksum != rx_data_buffer_[checksum_idx]) {
        result = RxResult::kErrFailedTail;
        resetDecodeProgress();
        break;
      }

      decodeRxData(kTxDataLen, rx_data_buffer_);

      resetDecodeProgress(true);
      result = RxResult::kOk;
    } while (0);
  } else {
    resetDecodeProgress(true);
    result = RxResult::kErrUndefined;
  }
  return result;
};

/**
 * @brief 重置解码进度。
 * 
 * 对解码状态进行重置，清空接收数据缓冲区，并重置其索引。
 */
void Vision::resetDecodeProgress(bool keep_rx_buffer)
{
  // 重置解码状态为等待开始帧
  rx_status_ = RxStatus::kWaitingHeaderSof;
  // 使用 0 填充接收数据缓冲区
  if (!keep_rx_buffer) {
    memset(rx_data_buffer_, 0, sizeof(rx_data_buffer_));
  }

  // 重置接收数据缓冲区的当前索引
  rx_data_buffer_idx_ = 0;
};

/**
 * @brief 解码接收到的数据
 *
 * 该函数解码接收到的数据，并更新 `rx_data_` 结构中的字段。
 *
 * @param len 接收到的数据长度
 * @param data 指向接收到的数据的指针
 */
void Vision::decodeRxData(size_t len, const uint8_t *data)
{
  if (len < kRxDataLen) {
    return;
  }

  int16_t temp_int16 = 0;
  temp_int16 = (int16_t)((data[3] << 8) | data[4]);
  rx_data_.pitch = ((float)temp_int16) * M_PI / 180.0f / 100.0f;

  temp_int16 = (int16_t)((data[5] << 8) | data[6]);
  rx_data_.yaw = ((float)temp_int16) * M_PI / 180.0f / 100.0f;
  if (isInHittingBuff()) {
    rx_data_.detected_ids = 0u;
    rx_data_.fly_time = (uint8_t)(data[7]);
  } else {
    rx_data_.detected_ids = (uint8_t)(data[7]);
    rx_data_.fly_time = 0u;
  }
  rx_data_.detected_state = DetectedState(data[8] >> 4);
  rx_data_.detected_color = TargetColor((data[8]) >> 2 & 0x03);
  rx_data_.shoot_flag = ShootFlag(data[8] & 0x03);
  rx_data_.vtm_x = data[9];
  rx_data_.vtm_y = data[10];
};

}  // namespace robot
