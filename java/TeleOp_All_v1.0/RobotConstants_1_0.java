package org.firstinspires.ftc.teamcode;

/**
 * 全局常量管理类
 * 集中管理所有机器人硬件配置参数、控制参数和常数
 * 便于统一维护和调试参数
 */
public class RobotConstants_1_0 {

    // ==================== 硬件配置常量 ====================

    // 底盘电机硬件映射名称
    public static final String CHASSIS_MOTOR_FRONT_LEFT_NAME = "lf";
    public static final String CHASSIS_MOTOR_FRONT_RIGHT_NAME = "rf";
    public static final String CHASSIS_MOTOR_BACK_LEFT_NAME = "lb";
    public static final String CHASSIS_MOTOR_BACK_RIGHT_NAME = "rb";

    // 子系统电机硬件映射名称
    public static final String SUBSYSTEM_MOTOR_INTAKE_NAME = "intake";
    public static final String SUBSYSTEM_MOTOR_LOAD_NAME = "load";
    public static final String SUBSYSTEM_MOTOR_SHOOTER_S1_NAME = "s1";
    public static final String SUBSYSTEM_MOTOR_SHOOTER_S2_NAME = "s2";

    // ==================== 底盘控制参数 ====================

    // 摇杆死区阈值（用于防止漂移）
    public static final double CHASSIS_JOYSTICK_DEADZONE = 0.1;

    // 右摇杆旋转灵敏度衰减系数
    public static final double CHASSIS_TURN_SENSITIVITY_FACTOR = 0.8;

    // 电机最大功率限制
    public static final double CHASSIS_MAX_MOTOR_POWER = 1.0;

    // 左摇杆非线性映射模式（默认启用）
    public static final boolean CHASSIS_NONLINEAR_MAPPING_DEFAULT = true;

    // ==================== 子系统控制参数 ====================

    // 扳机/按键死区阈值
    public static final double SUBSYSTEM_TRIGGER_DEADZONE = 0.1;

    // Intake（拾取）电机功率
    public static final double SUBSYSTEM_INTAKE_MOTOR_POWER = 1.0;
    public static final double SUBSYSTEM_INTAKE_REVERSE_POWER = -1.0;

    // Load（装填）电机功率
    public static final double SUBSYSTEM_LOAD_MOTOR_POWER = 1.0;

    // Shooter（发射）电机功率（s1正转，s2反转）
    public static final double SUBSYSTEM_SHOOTER_S1_POWER = 1.0;
    public static final double SUBSYSTEM_SHOOTER_S2_POWER = -1.0;

    // ==================== 发射模块PID闭环参数 ====================

    // ==================== 发射模块功率控制参数（已替代 RPM 控制） ====================

    // 发射器功率默认值和步长（范围 0.0 - 1.0）
    public static final double SHOOTER_POWER_DEFAULT = 0.8;    // 默认发射功率
    public static final double SHOOTER_POWER_STEP = 0.05;      // D-Pad 上下步长
    public static final double SHOOTER_POWER_STEP_LR = 0.02;   // D-Pad 左右步长
    public static final double SHOOTER_POWER_MIN = 0.0;
    public static final double SHOOTER_POWER_MAX = 1.0;

    // ==================== 遥测和显示参数 ====================

    // 遥测更新频率控制（可选）
    public static final long TELEMETRY_UPDATE_INTERVAL_MS = 50;  // 50ms更新一次

}
