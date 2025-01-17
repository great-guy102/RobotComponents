/**
 *******************************************************************************
 * @file      :module_state.hpp
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
#ifndef MODULE_STATE_HPP_
#define MODULE_STATE_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>
#include <string>

#include "fric_2motor.hpp"
#include "module_fsm.hpp"
/* Exported macro ------------------------------------------------------------*/

namespace robot {
/* Exported constants --------------------------------------------------------*/
// 专用状态枚举
/** 底盘工作模式 */
enum class ChassisWorkingMode : uint8_t {
  Depart, ///< 分离模式
  Follow, ///< 随动模式
  Gyro,   ///< 小陀螺模式
};

/** 云台工作模式 */
enum class GimbalWorkingMode {
  Normal,  ///< 正常模式
  PidTest, ///< PID 调试模式
};

/* Exported types ------------------------------------------------------------*/
typedef hello_world::module::PwrState PwrState;
typedef hello_world::module::CtrlMode CtrlMode;
typedef hello_world::module::Fric::WorkingMode ShooterWorkingMode;

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
inline std::string ChassisWorkingModeToStr(ChassisWorkingMode mode) {
  if (mode == ChassisWorkingMode::Depart)
    return "Depart";
  if (mode == ChassisWorkingMode::Follow)
    return "Follow";
  if (mode == ChassisWorkingMode::Gyro)
    return "Gyro";
  return "ErrCWM";
};

inline std::string GimbalWorkingModeToStr(GimbalWorkingMode mode) {
  if (mode == GimbalWorkingMode::Normal)
    return "Normal";
  return "ErrGWM";
};

inline std::string ShooterWorkingModeToStr(ShooterWorkingMode mode) {
  if (mode == ShooterWorkingMode::kShoot)
    return "Shoot";
  if (mode == ShooterWorkingMode::kStop)
    return "Stop";
  if (mode == ShooterWorkingMode::kBackward)
    return "Backward";
  return "ErrSWM";
};

} // namespace robot
#endif /* MODULE_STATE_HPP_ */
