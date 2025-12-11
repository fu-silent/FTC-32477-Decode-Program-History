package org.firstinspires.ftc.teamcode;

/**
 * FTC32477 TeleOp v2.3.0 - 常数和配置管理
 * 
 * 新增功能：
 * - 无头模式 (Field-Centric)
 * - IMU 航向重置
 */
public class RobotConstants_2_3 {
    
    // ========== 硬件名称配置 ==========
    
    // 底盘电机
    public static final String CHASSIS_MOTOR_FRONT_LEFT_NAME = "lf";
    public static final String CHASSIS_MOTOR_FRONT_RIGHT_NAME = "rf";
    public static final String CHASSIS_MOTOR_BACK_LEFT_NAME = "lb";
    public static final String CHASSIS_MOTOR_BACK_RIGHT_NAME = "rb";
    
    // 子系统电机
    public static final String SUBSYSTEM_INTAKE_MOTOR_NAME = "intake";
    public static final String SUBSYSTEM_LOAD_MOTOR_NAME = "load";
    public static final String SUBSYSTEM_SHOOTER1_MOTOR_NAME = "s1";
    public static final String SUBSYSTEM_SHOOTER2_MOTOR_NAME = "s2";
    
    // IMU 传感器
    public static final String IMU_SENSOR_NAME = "imu";
    
    // ========== 底盘控制参数 ==========
    
    // 死区（摇杆小于此值时忽略）
    public static final double CHASSIS_JOYSTICK_DEADZONE = 0.1;
    
    // ========== 发射系统参数 ==========
    
    // 电机编码器刻度常数
    public static final double SHOOTER_MOTOR_TICK_COUNT = 28;
    
    // 发射电机 PID 参数 (同步自 TeleOp_All_2_3)
    public static final double SHOOTER_PIDF_P = 250;
    public static final double SHOOTER_PIDF_I = 0;
    public static final double SHOOTER_PIDF_D = 100;
    public static final double SHOOTER_PIDF_F = 14;
    
    // 转速档位预设（RPM） (同步自 TeleOp_All_2_3)
    public static final int SHOOTER_RPM_LONG_RANGE = 1950;
    public static final int SHOOTER_RPM_TRIANGLE_SIDE = 1400;
    public static final int SHOOTER_RPM_TRIANGLE_BASE = 1200;
    public static final int SHOOTER_RPM_TRIANGLE_TOP = 1625;
    
    // 转速精度范围（RPM） (同步自 TeleOp_All_2_3)
    public static final int SHOOTER_RPM_ERROR_RANGE_LONG = 50;
    public static final int SHOOTER_RPM_ERROR_RANGE_SIDE = 50;
    public static final int SHOOTER_RPM_ERROR_RANGE_BASE = 50;
    public static final int SHOOTER_RPM_ERROR_RANGE_TOP = 50;
    
    // ========== 拾取和装填系统参数 ==========
    
    // 拾取电机功率
    public static final double INTAKE_FORWARD_POWER = 0.9;
    public static final double INTAKE_REVERSE_POWER = -0.9;
    public static final double INTAKE_STOP_POWER = 0.0;
    
    // 装填电机功率
    public static final double LOAD_FORWARD_POWER = 0.75;
    public static final double LOAD_REVERSE_POWER = -0.75;
    public static final double LOAD_STOP_POWER = 0.0;
    
    // ========== IMU 自动转向参数 ==========
    
    // 转向功率
    public static final double AUTO_TURN_POWER = 0.5;
    
    // 角度阈值（度，误差小于此值时停止转向）
    public static final double AUTO_TURN_HEADING_THRESHOLD = 2.0;
    
    // 转向 PID 参数
    public static final double AUTO_TURN_P_GAIN = 0.1;
    public static final double AUTO_TURN_I_GAIN = 0.0;
    public static final double AUTO_TURN_D_GAIN = 0.005;
    
    // 目标角度预设（度）
    public static final double AUTO_TURN_TARGET_RIGHT = 45.0;
    
    // ========== 杂项参数 ==========
    public static final int RUMBLE_DURATION_MS = 200; // 手柄震动时长
    
}
