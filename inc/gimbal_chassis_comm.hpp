/**
 *******************************************************************************
 * @file      :gimbal_chassis_comm.hpp
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
#ifndef HERO_COMPONENTS_GIMBAL_CHASSIS_COMM_HPP_
#define HERO_COMPONENTS_GIMBAL_CHASSIS_COMM_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "base.hpp"
#include "offline_checker.hpp"

#include "receiver.hpp"
#include "rfr_official_pkgs.hpp"
#include "transmitter.hpp"

#include "fric_2motor.hpp"
#include "module_fsm.hpp"

#include "module_state.hpp"
/* Exported macro ------------------------------------------------------------*/
namespace robot {
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class GimbalChassisComm : public hello_world::comm::Receiver,
                          public hello_world::comm::Transmitter {
public:
  typedef hello_world::referee::RfrId RobotId;
  typedef hello_world::referee::ids::TeamColor TeamColor;
  typedef hello_world::OfflineChecker OfflineChecker;
  typedef hello_world::module::PwrState PwrState;
  typedef hello_world::module::CtrlMode CtrlMode;
  typedef hello_world::module::Fric::WorkingMode ShooterWorkingMode;

  enum class CodePart : uint8_t {
    Chassis = 0,
    Gimbal = 1,
  };

  struct MainBoardData {
    // chassis to gimbal
    struct ChassisPart {
    } cp;

    // gimbal to chassis
    struct GimbalPart {
      bool is_gimbal_imu_ready = false; ///< 云台主控 IMU 工作状态
    } gp;
  };

  struct GimbalData {
    // chassis to gimbal
    struct ChassisPart {
      uint8_t rev_gimbal_cnt = 0; ///< 云台掉头标志计数器

      float yaw_delta = 0;   ///< 将归一化的角度增量值转换到-127~127
      float pitch_delta = 0; ///< 将归一化的角度增量值转换到-127~127

      CtrlMode ctrl_mode = CtrlMode::kManual; ///< 云台模块控制模式（工作状态为
                                              ///< kPwrStateWorking 时有效）

      GimbalWorkingMode working_mode =
          GimbalWorkingMode::Normal; ///< 云台模块的工作模式（工作状态为
                                     ///< kPwrStateWorking 时有效）
    } cp;

    // gimbal to chassis
    struct GimbalPart {
      PwrState pwr_state = PwrState::kDead; ///< 云台模块工作状态
      bool is_gimbal_motors_online = false; ///< 云台关节电机是否在线

      float yaw_fdb = 0.0f;   ///< 云台的当前偏航角度(关节空间)
      float pitch_fdb = 0.0f; ///< 云台的当前俯仰角度(关节空间)
      float yaw_ref = 0.0f;   ///< 云台的期望偏航角度(关节空间)
      float pitch_ref = 0.0f; ///< 云台的期望俯仰角度(关节空间)
    } gp;
  };

  struct ShooterData {
    // chassis to gimbal
    struct ChassisPart {
      void setShootFlag(bool flag) {
        if (flag) {
          shoot_count_++;
        }
      }
      void clearShootFlag() { last_shoot_count_ = shoot_count_; }
      bool shoot_flag(bool auto_clear = true) {
        bool res = shoot_count_ != last_shoot_count_;
        if (auto_clear) {
          clearShootFlag();
        }
        return res;
      }

      CtrlMode ctrl_mode = CtrlMode::kManual; ///< 发射机构模块控制模式

      ShooterWorkingMode working_mode =
          ShooterWorkingMode::kShoot; ///< 发射机构模块的工作模式

    private:
      uint8_t shoot_count_ = 0; ///< 射击次数，用于判断是否发送射击指令
      uint8_t last_shoot_count_ =
          0; ///< 上一次的射击次数，用于判断是否发送射击指令
    } cp;
    // gimbal to chassis
    struct GimbalPart {
      bool is_shooter_stuck = false;         ///< 摩擦轮是否卡住
      bool is_shooter_motors_online = false; ///< 摩擦轮电机是否在线
      uint8_t feed_stuck_state = 0;          ///< 拨盘卡住状态

      PwrState pwr_state = PwrState::kDead; ///< 发射机构模块工作状态
      float fric_spd_ref = 0;               ///< 摩擦轮转速期望值
      float fric_spd_fdb = 0.0f;            ///< 摩擦轮的转速反馈值
      float feed_ang_ref = 0;               ///< 拨盘角度期望值
      float feed_ang_fdb = 0.0f;            ///< 拨盘角度反馈值
    } gp;
  };

  struct RefereeData {
    // chassis to gimbal
    struct ChassisPart {
      bool is_rfr_on = false;
      bool is_rfr_gimbal_power_on = false;
      bool is_rfr_shooter_power_on = false;
      uint8_t rfr_bullet_shot_cnt = 0;
      RobotId robot_id = 3;
      float bullet_speed = 0;
      float shooter_heat = 0;
      float shooter_cooling = 0;
      float shooter_heat_limit = 0;
    } cp;
    // gimbal to chassis
  };

  struct VisionData {
    // Gimbal to Chassis
    struct GimbalPart {
      bool is_vision_online = false; ///< 视觉模块是否在线
      bool is_enemy_detected = false;

      uint8_t vtm_x = 0;
      uint8_t vtm_y = 0;
    } gp;
  };

  GimbalChassisComm(CodePart code_part, uint32_t chassis_id, uint32_t gimbal_id,
                    uint32_t offline_threshold = 5)
      : oc_(offline_threshold) {
    code_part_ = code_part;
    if (code_part_ == CodePart::Chassis) {
      tx_id_ = chassis_id;
      rx_id_ = gimbal_id;
    } else if (code_part_ == CodePart::Gimbal) {
      tx_id_ = gimbal_id;
      rx_id_ = chassis_id;
    } else {
      HW_ASSERT(false, "Invalid code part", __FILE__);
    }
    rx_ids_ = {rx_id_};
  };
  virtual ~GimbalChassisComm() = default;

  /**
   * @brief       获取接收器ID
   * @param        None
   * @retval       接收器ID
   * @note        None
   */
  virtual uint32_t rxId(void) const override { return rx_id_; };
  virtual const RxIds &rxIds(void) const override { return rx_ids_; };

  /**
   * @brief       解码
   * @param        data: 数据指针
   * @param        len: 数据长度
   * @retval       解码成功返回true，否则返回false
   * @note        None
   */
  virtual bool decode(size_t len, const uint8_t *data, uint32_t rx_id) override;

  /**
   * @brief       是否有更新数据
   * @retval       有更新数据返回true，否则返回false
   * @note        判断频率比 isOffline 更快
   */
  virtual bool isUpdate(void) const { return is_update_; };

  /**
   * @brief       清除更新标志
   * @retval       None
   * @note        None
   */
  virtual void clearUpdateFlag(void) { is_update_ = false; };

  /**
   * @brief       注册更新回调函数
   * @param        cb: 回调函数指针，在decode函数解码成功后被调用，
   * 不使用时传入nullptr
   * @retval       None
   * @note        None
   */
  virtual void registerUpdateCallback(pUpdateCallback cb) { update_cb_ = cb; };

  /**
   * @brief       获取发送器ID
   * @param        None
   * @retval       发送器ID
   * @note        None
   */
  virtual uint32_t txId(void) const override { return tx_id_; };

  /**
   * @brief       编码
   * @param        len: 缓冲区长度
   * @param        data: 缓冲区指针
   * @retval       编码成功返回true，否则返回false
   * @note        编码成功后len需修改为编码后的数据长度
   */
  virtual bool encode(size_t &len, uint8_t *data) override;

  /**
   * @brief       发送成功回调
   *
   * 成功发送后调用，用于统计发送成功的次数
   * @retval      None
   */
  void txSuccessCb(void) override { transmit_success_cnt_++; }

  // 直接访问数据
  MainBoardData &main_board_data() { return main_board_data_; }
  GimbalData &gimbal_data() { return gimbal_data_; }
  ShooterData &shooter_data() { return shooter_data_; }
  RefereeData &referee_data() { return referee_data_; }
  VisionData &vision_data() { return vision_data_; }

  bool isOffline() { return oc_.isOffline(); }
  void setOfflineThreshold(uint32_t threshold) {
    oc_.set_offline_tick_thres(threshold);
  }
  void setTxId(uint32_t tx_id) { tx_id_ = tx_id; };
  void setRxId(uint32_t rx_id) { rx_id_ = rx_id; };

private:
  void encodeG2C(uint8_t tx_data[8]);
  void decodeG2C(const uint8_t rx_data[8]);
  void encodeC2G(uint8_t tx_data[8]);
  void decodeC2G(const uint8_t rx_data[8]);

  CodePart code_part_ =
      CodePart::Chassis; ///< 代码所在部分，云台还是底盘，决定编解码方式
  size_t g2c_seq_ = 0;

  // 解码相关
  uint32_t rx_id_ = 0x04;                   ///< 接收的CAN消息ID
  bool is_update_ = false;                  ///< 是否有更新数据
  pUpdateCallback update_cb_ = nullptr;     ///< 更新回调函数
  OfflineChecker oc_ = OfflineChecker(100); ///< 离线检测器

  // 编码相关
  uint32_t tx_id_ = 0x03;             ///< 发送的CAN消息ID
  uint32_t transmit_success_cnt_ = 0; ///< 发送成功次数
  RxIds rx_ids_ = {rx_id_};           ///< 接收器ID列表

  // 所有数据
  MainBoardData main_board_data_;
  GimbalData gimbal_data_;
  ShooterData shooter_data_;
  RefereeData referee_data_;
  VisionData vision_data_;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
} // namespace robot
#endif /* HERO_COMPONENTS_GIMBAL_CHASSIS_COMM_HPP_ */
