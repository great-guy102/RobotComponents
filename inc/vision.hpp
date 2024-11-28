/** 
 *******************************************************************************
 * @file      : vision.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HERO_COMPONENTS_VISION_HPP_
#define HERO_COMPONENTS_VISION_HPP_

/* Includes ------------------------------------------------------------------*/
#include <algorithm>
#include <cmath>

#include "offline_checker.hpp"
#include "receiver.hpp"
#include "transmitter.hpp"
/* Exported macro ------------------------------------------------------------*/

namespace robot
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class Vision : public hello_world::comm::Receiver, public hello_world::comm::Transmitter
{
 public:
  typedef hello_world::OfflineChecker OfflineChecker;

  enum class RxStatus : uint8_t {
    kWaitingHeaderSof = 0u,  ///< 等待 SOF
    kWaitingHeaderCplt,      ///< 等待帧头结束
    kWaitingTailCplt,        ///< 等待帧尾结束
  };

  enum class RxResult : uint8_t {
    kErrUndefined,       ///< 未定义的错误
    kErrNoDataInput,     ///< 未开始接收数据，一直等不到 SOF 0xAA
    kErrFailedHeader,    ///< 帧头校验失败，帧头不为 0xAA 0xBB 0xCC
    kErrFailedTail,      ///< 帧尾校验失败，帧尾不为 0xFF
    kErrFailedChecksum,  ///< 校验和校验失败
    kHandlingWaitSof,    ///< 处理中-等待 SOF
    kHandlingHeader,     ///< 处理中-正在接收帧头数据
    kHandlingWaitTail,   ///< 处理中-等待帧尾
    kOk,                 ///< 校验通过，解包成功
  };

  enum class WorkState : uint8_t {
    kStandby = 0x00,           ///< 待机
    kNormal = 0x01,            ///< 普通自瞄
    kAntiTop = 0x02,           ///< 反陀螺自瞄
    kSmallBuff = 0x03,         ///< 识别小符
    kBigBuff = 0x04,           ///< 识别大符
    kCounterSmallBuff = 0x05,  ///< 识别小符（计数模式）
    kCounterBigBuff = 0x06,    ///< 识别大符（计数模式）
    kFarshoot = 0x07,          ///< 吊射
    kReboot = 0xDD,            ///< 重启
    kDebug = 0xEE,             ///< 调试
  };

  enum class RobotColor : uint8_t {
    kUndefined = 0u,  ///< 未定义
    kRed = 1u,        ///< 红方
    kBlue = 2u,       ///< 蓝方
  };

  enum class TargetColor : uint8_t {
    kGray = 0u,    ///< 无灯条装甲板
    kBlue = 1u,    ///< 蓝色装甲板，当己方机器人为红方时自动设置
    kRed = 2u,     ///< 红色装甲板，当己方机器人为蓝方时自动设置
    kPurple = 3u,  ///< 紫色装甲板
  };

  enum class Balance : uint8_t {
    kNoBalance = 0u,  ///< 没有平衡步兵
    kStandard3 = 3u,  ///< 3 号步兵
    kStandard4 = 4u,  ///< 4 号步兵
    kStandard5 = 5u,  ///< 5 号步兵
  };

  enum class DetectedState : uint8_t {
    kNone = 0u,  ///< 未检测到目标
    kDetected,   ///< 检测到目标
    kAimed,      ///< 瞄准目标
  };

  enum class ShootFlag : uint8_t {
    kNoShoot = 0u,     ///< 不射击
    kShootOnce,        ///< 射击一次
    kShootContinuous,  ///< 连续射击
  };

  struct TxData {
    WorkState work_state = WorkState::kStandby;  ///< 工作状态
    TargetColor tgt_color = TargetColor::kGray;  ///< 目标颜色
    Balance balance = Balance::kNoBalance;       ///< 平衡步兵编号

    // 发送给视觉的 XYZ 固定角，左手系，Z 轴竖直向上，X 轴指向前方，Y 轴指向右方
    float roll = 0.0f;   ///< 绕 X 轴旋转角度，单位：rad
    float pitch = 0.0f;  ///< 绕 Y 轴旋转角度，单位：rad
    float yaw = 0.0f;    ///< 绕 Z 轴旋转角度，单位：rad

    float blt_spd = 15.5f;  ///< 发送给视觉的子弹速度，单位：m/s
  };

  struct RxData {
    ShootFlag shoot_flag = ShootFlag::kNoShoot;  ///< 自动射击标志位

    uint8_t detected_ids = 0u;  ///< 识别到的目标 id
    uint8_t fly_time = 0u;      ///< 飞行时间，单位：ms

    DetectedState detected_state = DetectedState::kNone;  ///< 识别到的目标数量
    TargetColor detected_color = TargetColor::kGray;      ///< 识别到的目标颜色

    // 视觉发送的 XYZ 固定角，左手系，Z 轴竖直向上，X 轴指向前方，Y 轴指向右方
    float pitch = 0.0f;  ///< 绕 Y 轴旋转角度，单位：rad
    float yaw = 0.0f;    ///< 绕 Z 轴旋转角度，单位：rad

    uint8_t vtm_x = 0u;  ///< 瞄准目标在图传中的 X 轴坐标，单位：像素
    uint8_t vtm_y = 0u;  ///< 瞄准目标在图传中的 Y 轴坐标，单位：像素
  };

  struct Config {
    float default_blt_spd = 15.5f;     ///< 默认子弹速度，单位：m/s
    float blt_spd_filter_beta = 0.9f;  ///< 子弹速度滤波系数，[0, 1]，0 表示不滤波
    uint32_t offline_thres = 100u;     ///< 视觉离线阈值，单位：ms

    float hfov = 0.52f;  ///< 水平视场角，单位：rad，默认 30°，对应 YAW 轴
    float vfov = 0.52f;  ///< 垂直视场角，单位：rad，默认 30°，对应 PITCH 轴
  };

  static constexpr uint32_t kRxDataLen = 13;  ///< 视觉接收数据长度
  static constexpr uint32_t kTxDataLen = 13;  ///< 视觉发送数据长度

  Vision(void)
  {
    setConfig(Config());
    resetRxData();
    resetTxData();
  };
  Vision(const Config &cfg)
  {
    setConfig(cfg);
    resetRxData();
    resetTxData();
  };
  Vision(const Vision &) = default;
  Vision &operator=(const Vision &other) = default;
  Vision(Vision &&other) = default;
  Vision &operator=(Vision &&other) = default;

  virtual ~Vision(void) = default;

  /* 重载方法 */

  virtual uint32_t rxId(void) const override { return 0u; }

  /**
   * @brief 解码输入数据并处理
   *
   * 该函数解码输入数据，并调用处理函数处理每个字节。如果解码成功，则更新状态并调用回调函数。
   *
   * @param len 输入数据的长度
   * @param data 指向输入数据的指针
   * @return true 如果成功解码并处理了至少一个数据帧
   * @return false 如果输入数据为空或长度为零，或者解码失败
   */
  virtual bool decode(size_t len, const uint8_t *data) override;

  virtual bool isUpdate(void) const override { return is_updated_; }

  virtual void clearUpdateFlag(void) override { is_updated_ = false; }

  virtual void registerUpdateCallback(pUpdateCallback cb) override { update_cb_ = cb; }

  virtual uint32_t txId(void) const override { return 0u; }

  /**
   * @brief 编码数据并存储到输出缓冲区
   *
   * 该函数将Vision对象中的数据编码为特定格式的数据包，并存储到输出缓冲区中。
   * 编码后的数据包长度为kTxDataLen。
   *
   * @param len 输出参数，表示编码后的数据长度
   * @param data 输出参数，指向存储编码后数据的缓冲区
   * @return true 如果编码成功
   * @return false 如果数据长度不足
   */
  virtual bool encode(size_t &len, uint8_t *data) override;

  virtual void txSuccessCb(void) override { tx_success_cnt_++; }

  /* 功能函数 */

  /** 重置接收数据 */
  void resetRxData(void);

  /**
   * @brief 重置发送数据
   *
   * 该函数将发送数据结构 `tx_data_` 重置为默认值。其中弹速会被重置为 `default_blt_spd_`，其他字段则见 `Vision::TxData`
   */
  void resetTxData(void);

  /**
   * @brief 检查是否检测到目标
   *
   * 该函数检查接收到的数据中是否检测到目标，并且目标状态是否不为 `kNone`，同时检查是否处于离线状态。
   *
   * @retval true 如果检测到目标且目标状态不为 `kNone`，并且不处于离线状态
   * @return false 其他情况
   */
  bool isTargetDetected(void) { return rx_data_.detected_ids != 0 && rx_data_.detected_state != DetectedState::kNone && (!isOffline()); };

  bool isOffline(void) { return oc_.isOffline(); };

  bool isDetectedInView(float pitch, float yaw) const
  {
    return (std::fabs(rx_data_.pitch - pitch) < half_vfov_) && (std::fabs(rx_data_.yaw - yaw) < half_hfov_);
  }

  bool isDetectedInView(void) const { return isDetectedInView(tx_data_.pitch, tx_data_.yaw); }

  bool isInHittingBuff(void) const { return tx_data_.work_state == WorkState::kSmallBuff || tx_data_.work_state == WorkState::kBigBuff; };

  bool isProcessByteNoErr(void) const
  {
    return rx_result_ == RxResult::kHandlingWaitTail    ///< 处理中-等待帧尾
           || rx_result_ == RxResult::kHandlingHeader   ///< 处理中-正在接收帧头数据
           || rx_result_ == RxResult::kOk               ///< 校验通过，解包成功
           || rx_result_ == RxResult::kHandlingWaitSof  ///< 处理中-等待 SOF
        ;
  };

  bool isProcessByteOk(void) const { return rx_result_ == RxResult::kOk; };

  // 接口函数
  // 配置相关
  /**
   * @brief 设置视觉配置
   *
   * 该函数设置视觉系统的配置参数，并对某些参数进行约束。
   *
   * @param cfg 配置信息
   *
   * @note bullet_speed_filter_beta 会被限制在 0.0 到 1.0 之间。
   */
  void setConfig(const Config &cfg)
  {
    setDefaultBulletSpeed(cfg.default_blt_spd);
    setBulletSpeedFilterBeta(cfg.blt_spd_filter_beta);
    setOfflineThreshold(cfg.offline_thres);
  };

  float getDefaultBulletSpeed(void) const { return default_blt_spd_; };
  void setDefaultBulletSpeed(float blt_spd) { default_blt_spd_ = blt_spd; };

  float getBulletSpeedFilterBeta(void) const { return blt_spd_filter_beta_; };
  void setBulletSpeedFilterBeta(float beta) { blt_spd_filter_beta_ = std::clamp(beta, 0.0f, 1.0f); };

  uint32_t getOfflineTickTrheshold(void) const { return oc_.get_offline_tick_thres(); };
  void setOfflineThreshold(uint32_t threshold) { oc_.set_offline_tick_thres(threshold); };

  float getHorizontalFov(void) const { return half_hfov_ * 2.0f; };
  void setHorizontalFov(float fov) { half_hfov_ = fov / 2.0f; };

  float getHalfHorizontalFov(void) const { return half_hfov_; };
  void setHalfHorizontalFov(float fov) { half_hfov_ = fov; };

  float getVerticalFov(void) const { return half_vfov_ * 2.0f; };
  void setVerticalFov(float fov) { half_vfov_ = fov / 2.0f; };

  float getHalfVerticalFov(void) const { return half_vfov_; };
  void setHalfVerticalFov(float fov) { half_vfov_ = fov; };

  void setFov(float fov)
  {
    setHorizontalFov(fov);
    setVerticalFov(fov);
  };

  void setFov(float hfov, float vfov)
  {
    setHorizontalFov(hfov);
    setVerticalFov(vfov);
  };

  // 发送给视觉的数据

  void setWorkState(WorkState work_state) { tx_data_.work_state = work_state; };
  WorkState getWorkState(void) const { return tx_data_.work_state; };

  void setRobotColor(RobotColor color)
  {
    if (color == RobotColor::kRed) {
      setTargetColor(TargetColor::kBlue);
    } else if (color == RobotColor::kBlue) {
      setTargetColor(TargetColor::kRed);
    }
    setTargetColor(TargetColor::kGray);
  }
  void setTargetColor(TargetColor color) { tx_data_.tgt_color = color; };
  TargetColor getTargetColor(void) const { return tx_data_.tgt_color; };

  void setBalanceNumber(Balance balance) { tx_data_.balance = balance; };
  Balance getBalance(void) const { return tx_data_.balance; };

  void setPose(float roll, float pitch, float yaw, bool is_left_hand_system = false)
  {
    tx_data_.roll = roll;
    tx_data_.pitch = is_left_hand_system ? pitch : -pitch;
    tx_data_.yaw = yaw;
  };
  float getPoseRoll(bool is_left_hand_system = false) const { return tx_data_.roll; };
  float getPosePitch(bool is_left_hand_system = false) const { return is_left_hand_system ? tx_data_.pitch : -tx_data_.pitch; };
  float getPoseYaw(bool is_left_hand_system = false) const { return tx_data_.yaw; };

  bool setBulletSpeed(float blt_spd)
  {
    if (tx_data_.blt_spd == blt_spd) {
      return false;
    }
    float beta = blt_spd_filter_beta_;
    tx_data_.blt_spd = beta * tx_data_.blt_spd + (1 - beta) * blt_spd;
    return true;
  };
  float getBulletSpeed(void) const { return tx_data_.blt_spd; };

  // 视觉发送的数据

  const RxData &getRxData(void) const { return rx_data_; };

  ShootFlag getShootFlag(void) const { return rx_data_.shoot_flag; };

  uint8_t getDetectedId(void) const { return rx_data_.detected_ids; };

  uint8_t getFlyTime(void) const { return rx_data_.fly_time; };

  DetectedState getDetectedState(void) const { return rx_data_.detected_state; };

  TargetColor getDetectedColor(void) const { return rx_data_.detected_color; };

  float getPoseRefPitch(bool is_left_hand_system = false) const { return is_left_hand_system ? rx_data_.pitch : -rx_data_.pitch; };

  float getPoseRefYaw(bool is_left_hand_system = false) const { return rx_data_.yaw; };

  uint8_t getVtmX(void) const { return rx_data_.vtm_x; };

  uint8_t getVtmY(void) const { return rx_data_.vtm_y; };

  // 杂项
  uint32_t getEncodeSuccessCnt(void) const { return encode_success_cnt_; };
  uint32_t getEncodeFailCnt(void) const { return encode_fail_cnt_; };
  uint32_t getDecodeSuccessCnt(void) const { return decode_success_cnt_; };
  uint32_t getDecodeFailCnt(void) const { return decode_fail_cnt_; };

 private:
  RxResult processByte(uint8_t byte);

  void resetDecodeProgress(bool keep_rx_buffer = false);

  void decodeRxData(size_t len, const uint8_t *data);

  //  配置参数
  float default_blt_spd_ = 0.0f;      ///< 默认子弹速度，单位：m/s
  float blt_spd_filter_beta_ = 0.9f;  ///< 子弹速度滤波系数

  float half_hfov_ = 0.52f;  ///< 相机水平视场角的一半，单位：rad，对应 YAW
  float half_vfov_ = 0.52f;  ///< 相机垂直视场角的一半，单位：rad，对应 PITCH

  // rx
  RxResult rx_result_ = RxResult::kErrNoDataInput;    ///< 接收结果
  RxStatus rx_status_ = RxStatus::kWaitingHeaderSof;  ///< 接收状态

  uint8_t rx_data_buffer_idx_ = 0;            ///< 接收数据索引
  uint8_t rx_data_buffer_[kRxDataLen] = {0};  ///< 接收数据缓存
  RxData rx_data_;                            ///< 接收到并被解包的视觉数据

  uint32_t n_rx_frames_ = 0;  ///< 接收帧数，debug 用

  // tx
  bool is_updated_ = false;              ///< 接收数据是否更新
  pUpdateCallback update_cb_ = nullptr;  ///< 更新回调函数

  TxData tx_data_;                            ///< 发送给视觉的数据
  uint8_t tx_data_buffer_[kTxDataLen] = {0};  ///< 发给视觉的数据缓存，debug 用

  // 通讯杂项
  OfflineChecker oc_ = OfflineChecker(200);  ///< 离线检测器查器

  uint32_t encode_success_cnt_ = 0;  ///< 编码成功次数，debug 用
  uint32_t encode_fail_cnt_ = 0;     ///< 编码失败次数，debug 用
  uint32_t decode_success_cnt_ = 0;  ///< 解码成功次数，debug 用
  uint32_t decode_fail_cnt_ = 0;     ///< 解码失败次数，debug 用
  uint32_t tx_success_cnt_ = 0;      ///< 发送成功次数，debug 用
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace robot
#endif /* HERO_COMPONENTS_VISION_HPP_ */
