package org.firstinspires.ftc.teamcode;

/**
 * FTC32477 TeleOp v2.1.0 - 常数和配置管理
 * 
 * 新增功能：
 * - IMU 自动转向控制
 * - 双发射电机独立控制
 * - 多转速档位预设
 */
public class RobotConstants_2_1 {
    
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
    
    // 转向灵敏度（0-1，越大越灵敏）
    public static final double TURN_SENSITIVITY_FACTOR = 0.8;
    
    // ========== 发射系统参数 ==========
    
    // 电机编码器刻度常数
    public static final double SHOOTER_MOTOR_TICK_COUNT = 28;
    
    // 发射电机 PID 参数
    public static final double SHOOTER_PIDF_P = 135;
    public static final double SHOOTER_PIDF_I = 0;
    public static final double SHOOTER_PIDF_D = 80;
    public static final double SHOOTER_PIDF_F = 14;
    
    // 转速档位预设（RPM）
    public static final int SHOOTER_RPM_LONG_RANGE = 1800;      // 超远距离（D-Pad 右）
    public static final int SHOOTER_RPM_TRIANGLE_SIDE = 1400;   // 三角形腰部（D-Pad 左）
    public static final int SHOOTER_RPM_TRIANGLE_BASE = 1200;   // 三角形底部（D-Pad 下）
    public static final int SHOOTER_RPM_TRIANGLE_TOP = 1650;    // 三角形顶点（D-Pad 上）
    
    // 转速精度范围（RPM）
    public static final int SHOOTER_RPM_ERROR_RANGE_LONG = 200;     // 超远精度范围
    public static final int SHOOTER_RPM_ERROR_RANGE_SIDE = 200;     // 腰部精度范围
    public static final int SHOOTER_RPM_ERROR_RANGE_BASE = 200;     // 底部精度范围
    public static final int SHOOTER_RPM_ERROR_RANGE_TOP = 200;      // 顶点精度范围
    
    // ========== 拾取和装填系统参数 ==========
    
    // 拾取电机功率
    public static final double INTAKE_FORWARD_POWER = 0.8;
    public static final double INTAKE_REVERSE_POWER = -0.8;
    public static final double INTAKE_STOP_POWER = 0.0;
    
    // 装填电机功率
    public static final double LOAD_FORWARD_POWER = 0.55;
    public static final double LOAD_REVERSE_POWER = -0.55;
    public static final double LOAD_STOP_POWER = 0.0;
    
    // ========== IMU 自动转向参数 ==========
    
    // 转向功率
    public static final double AUTO_TURN_POWER = 0.5;
    
    // 角度阈值（度，误差小于此值时停止转向）
    public static final double AUTO_TURN_HEADING_THRESHOLD = 2.0;
    
    // 转向 PID 参数
    public static final double AUTO_TURN_P_GAIN = 0.1;       // 比例增益
    public static final double AUTO_TURN_I_GAIN = 0.0;       // 积分增益
    public static final double AUTO_TURN_D_GAIN = 0.005;     // 微分增益
    
    // 目标角度预设（度）
    public static final double AUTO_TURN_TARGET_RIGHT = 45.0;   // 右转 45 度（右肩键）
    public static final double AUTO_TURN_TARGET_LEFT = -45.0;   // 左转 45 度
    
    // ========== 状态机参数 ==========
    
    // 发射状态控制
    public static final int STATE_IDLE = 0;              // 待机
    public static final int STATE_PREPARING = 1;         // 准备中
    public static final int STATE_READY_TO_FIRE = 2;     // 准备好发射
    public static final int STATE_FIRING = 3;            // 发射中
    
    // ========== 电机方向配置 ==========
    
    // 底盘电机方向
    public static final String[] CHASSIS_DIRECTIONS = {
        "FORWARD",    // lf
        "REVERSE",    // rf
        "REVERSE",    // lb
        "FORWARD"     // rb
    };
    
    // 子系统电机方向
    public static final String[] SUBSYSTEM_DIRECTIONS = {
        "FORWARD",    // intake
        "REVERSE",    // load
        "REVERSE",    // shooter1
        "REVERSE"     // shooter2
    };
}
