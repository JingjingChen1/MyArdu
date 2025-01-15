#include "Copter.h"

#if RUN_CUSTOM_FLIGHT_TASK 

    // 参数配置
    constexpr float TAKEOFF_ALTITUDE = 5.0f;  // 起飞高度 (米)
    constexpr float MAX_SPEED = 1.0f;         // 飞行最大速度 (米/秒)
    constexpr uint32_t FORWARD_DURATION = 6000; // 向前飞行持续时间 (毫秒)

    // 状态变量
    static uint8_t flight_phase = 0;  // 飞行阶段：0=起飞，1=悬停，2=向前飞行，3=降落
    static uint64_t start_time = 0;   // 当前阶段的开始时间


    // 主任务函数: 负责执行自定义飞控任务
    void Copter::my_flight_task_update() {

        switch (flight_phase)
        {
        case 0:
        { // 起飞阶段
            if (!my_takeoff())
            {
                return;
            }
            flight_phase = 1;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FlightTaskTest: The copter is taking off...");
            break;
        }
        case 1:
        { // 悬停阶段，等待达到目标高度
            float current_position_D;
            ahrs.get_relative_position_D_home(current_position_D);

            // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FlightTaskTest: current altitude %f m", current_position_D);

            if (fabsf(-current_position_D - TAKEOFF_ALTITUDE) < 0.5f)
            { // 判定达到目标高度（允许 0.5 米误差）
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FlightTaskTest: reach target altitude, start hover...");
                my_hover();
                start_time = AP_HAL::millis64(); // 记录悬停开始时间
                flight_phase = 2;
            }
            
            break;
        }
        case 2:
        { // 悬停阶段
            my_hover();
            if (AP_HAL::millis64() - start_time > 5000)
            { // 悬停 5 秒
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FlightTaskTest: start to fly forward...");
                start_time = AP_HAL::millis64(); // 记录向前飞行的开始时间
                flight_phase = 3;
            }
            break;
        }
        case 3:
        { // 向前飞行阶段
            my_fly_forward();
            if (AP_HAL::millis64() - start_time > FORWARD_DURATION)
            { // 飞行 6 秒
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FlightTaskTest: start to land...");
                flight_phase = 4;
            }
            break;
        }
        case 4:
        { // 降落阶段
            my_hover(); // 短暂悬停以稳定
            if (my_land())
            {
                flight_phase = 5;
            }
            break;
        }
        case 5:
        { // 任务完成
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FlightTaskTest: task finished!");
            break;
        }
        default:
            break;
        }

    } 


    // 起飞函数
    bool Copter::my_takeoff() {

        //检查电机解锁情况
        if (copter.motors->armed()) {
    
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FlightTaskTest: Copter is armed!");

            // 切换到 GUIDED 模式
            if (!copter.set_mode(Mode::Number::GUIDED, ModeReason::STARTUP)) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "FlightTaskTest: unable to change mode to GUIDED!");
                return false;
            }           

            // 启动起飞到指定高度
            if (!copter.start_takeoff(TAKEOFF_ALTITUDE)) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "FlightTaskTest: unable to takeoff!");
                return false;
            }

            return true;

        }else{
            return false;
        }

    }

    // 悬停函数
    void Copter::my_hover() {
        // 设置目标速度为 0，保持悬停
        copter.set_target_velocity_NED(Vector3f(0.0f, 0.0f, 0.0f));
        // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FlightTaskTest: hovering...");
    }

    // 向前飞行函数
    void Copter::my_fly_forward() {
        // 设置目标速度向前飞行，保持高度
        copter.set_target_velocity_NED(Vector3f(MAX_SPEED, 0.0f, 0.0f));
        // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FlightTaskTest: flying forward...");
    }

    // 降落函数
    bool Copter::my_land() {
        // 切换到 LAND 模式
        if (!copter.set_mode(Mode::Number::LAND, ModeReason::UNKNOWN)) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "FlightTaskTest: unable to change mode to LAND!");
            return false;
        }

        // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FlightTaskTest: landing...");
        return true;
    }

#endif