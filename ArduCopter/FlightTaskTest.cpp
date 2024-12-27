
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include "Copter.h"

namespace chenjingjing {


#define TIME_RUN_TASK_ONCE 200 //任务执行频率（毫秒）

// 参数配置
constexpr float TAKEOFF_ALTITUDE = 20.0f;  // 起飞高度 (米)
constexpr float MAX_SPEED = 16.0f;         // 飞行最大速度 (米/秒)
constexpr uint32_t FORWARD_DURATION = 6000; // 向前飞行持续时间 (毫秒)

// 状态变量
static uint8_t flight_phase = 0;  // 飞行阶段：0=起飞，1=悬停，2=向前飞行，3=降落
static uint64_t start_time = 0;   // 当前阶段的开始时间



// 起飞函数
bool takeoff() {
    if (!vehicle.set_mode(MODE_GUIDED)) { // 切换到 GUIDED 模式
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "无法切换到 GUIDED 模式");
        return false;
    }
    if (!ap.arm_manager.arm(true)) { // 解锁电机 (ARM)
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "无法解锁电机");
        return false;
    }
    copter.takeoff_start(TAKEOFF_ALTITUDE); // 启动起飞到指定高度
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "起飞中...");
    return true;
}

// 悬停函数
void hover() {
    copter.set_target_velocity_xy(0.0f, 0.0f); // 停止 X 和 Y 方向速度
    copter.set_target_climb_rate(0.0f);       // 保持当前高度
}

// 向前飞行函数
void fly_forward() {
    copter.set_target_velocity_xy(MAX_SPEED, 0.0f); // 设置 X 方向速度（向前飞行），Y 方向速度为 0
    copter.set_target_climb_rate(0.0f);            // 保持当前高度
}

// 降落函数
bool land() {
    if (!ap.vehicle.set_mode(MODE_LAND)) { // 切换到 LAND 模式
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "无法切换到 LAND 模式");
        return false;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "降落中...");
    return true;
}



// 定义全局变量来记录上一次任务执行的时间点
static uint64_t last_task_time = 0;  // 上一次任务执行时间（毫秒）

// 主任务函数: 负责执行自定义飞控任务
void flight_task_update() {

    // 获取当前时间
    uint64_t current_time = AP_HAL::millis64();

    if (current_time - last_task_time >= TIME_RUN_TASK_ONCE)
    {
        last_task_time = current_time; // 更新上一次执行时间

        switch (flight_phase)
        {
        case 0:
        { // 起飞阶段
            if (!takeoff())
            {
                return;
            }
            flight_phase = 1;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "起飞阶段开始...");
            break;
        }
        case 1:
        {                                                     // 悬停阶段，等待达到目标高度
            float current_altitude = ahrs.get_relative_alt(); // 获取当前相对高度（米）
            if (current_altitude >= TAKEOFF_ALTITUDE - 0.5f)
            { // 判定达到目标高度（允许 0.5 米误差）
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "达到目标高度，开始悬停...");
                hover();
                start_time = AP_HAL::millis64(); // 记录悬停开始时间
                flight_phase = 2;
            }
            break;
        }
        case 2:
        { // 悬停阶段
            hover();
            if (AP_HAL::millis64() - start_time > 5000)
            { // 悬停 5 秒
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "开始向前飞行...");
                start_time = AP_HAL::millis64(); // 记录向前飞行的开始时间
                flight_phase = 3;
            }
            break;
        }
        case 3:
        { // 向前飞行阶段
            fly_forward();
            if (AP_HAL::millis64() - start_time > FORWARD_DURATION)
            { // 飞行 6 秒
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "准备降落...");
                flight_phase = 4;
            }
            break;
        }
        case 4:
        {            // 降落阶段
            hover(); // 短暂悬停以稳定
            if (land())
            {
                flight_phase = 5;
            }
            break;
        }
        case 5:
        { // 任务完成
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "飞行任务完成");
            break;
        }
        default:
            break;
        }
    }
}


} //chenjingjing