package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * 控制输入处理类
 * 负责处理手柄输入、按键映射、边沿检测
 * 职责：
 * - 读取Xbox手柄的摇杆、按键、扳机输入
 * - 实现按键的边沿检测（防止重复触发）
 * - 提供清晰的输入访问接口
 * - 维护按键状态历史（用于边沿检测）
 */
public class ControlInputManager_1_0 {

    // ==================== 输入对象 ====================
    private Gamepad gamepadPrimary;

    // ==================== 按键状态记录（用于边沿检测） ====================
    private boolean stateYButtonPrev = false;
    private boolean stateDpadUpPrev = false;
    private boolean stateDpadDownPrev = false;
    private boolean stateDpadLeftPrev = false;
    private boolean stateDpadRightPrev = false;

    // ==================== 模式状态 ====================
    private boolean isChassisMappingNonlinear = RobotConstants_1_0.CHASSIS_NONLINEAR_MAPPING_DEFAULT;

    // ==================== 构造函数 ====================
    public ControlInputManager_1_0(Gamepad gamepad) {
        this.gamepadPrimary = gamepad;
    }

    // ==================== 底盘控制输入（摇杆） ====================

    /**
     * 获取前后移动输入（左摇杆Y轴）
     * 向前为负，向后为正（Xbox标准）
     * @return -1.0 到 1.0
     */
    public double getChassisDriveFBInput() {
        // 取反是因为摇杆向前为负值
        return -gamepadPrimary.left_stick_y;
    }

    /**
     * 获取左右平移输入（右摇杆X轴）
     * @return -1.0 到 1.0
     */
    public double getChassisStrafeLRInput() {
        return gamepadPrimary.right_stick_x;
    }

    /**
     * 获取旋转输入（左摇杆X轴）
     * 左摇杆需要降低灵敏度，乘以衰减系数
     * @return -1.0 到 1.0，已应用灵敏度衰减
     */
    public double getChassisRotateCWInput() {
        return gamepadPrimary.left_stick_x * RobotConstants_1_0.CHASSIS_TURN_SENSITIVITY_FACTOR;
    }

    /**
     * 检查是否启用非线性映射模式
     */
    public boolean isChassisMappingNonlinearEnabled() {
        return isChassisMappingNonlinear;
    }

    /**
     * 设置非线性映射模式
     */
    public void setChassisMappingNonlinear(boolean enabled) {
        isChassisMappingNonlinear = enabled;
    }

    /**
     * 切换非线性映射模式
     */
    public void toggleChassisMappingMode() {
        isChassisMappingNonlinear = !isChassisMappingNonlinear;
    }

    // ==================== 拾取系统(Intake)控制输入 ====================

    /**
     * 检查A键是否按下（拾取正向）
     */
    public boolean isIntakeForwardRequested() {
        return gamepadPrimary.a;
    }

    /**
     * 检查B键是否按下（拾取反向）
     */
    public boolean isIntakeReverseRequested() {
        return gamepadPrimary.b;
    }

    // ==================== 装填系统(Load)控制输入 ====================

    /**
     * 获取左扳机值（装填系统）
     * @return 0.0 到 1.0
     */
    public double getLoadTriggerValue() {
        return gamepadPrimary.left_trigger;
    }

    /**
     * 检查左扳机是否被按下（超过死区）
     */
    public boolean isLoadRequested() {
        return getLoadTriggerValue() > RobotConstants_1_0.SUBSYSTEM_TRIGGER_DEADZONE;
    }

    /**
     * 检查左肩键(lb)是否被按下（装填反转）
     */
    public boolean isLoadReverseRequested() {
        return gamepadPrimary.left_bumper;
    }

    // ==================== 发射系统(Shooter)控制输入 ====================

    /**
     * 获取右扳机值（发射系统）
     * @return 0.0 到 1.0
     */
    public double getShooterTriggerValue() {
        return gamepadPrimary.right_trigger;
    }

    /**
     * 检查右扳机是否被按下（超过死区）
     */
    public boolean isShooterRequested() {
        return getShooterTriggerValue() > RobotConstants_1_0.SUBSYSTEM_TRIGGER_DEADZONE;
    }

    // ==================== 发射速度调节输入 ====================

    /**
     * 检查D-Pad上是否被按下（用于增加转速）
     * 使用边沿检测：只在按下瞬间返回true
     * @return 本帧是否从未按下变为按下
     */
    public boolean isShooterPowerIncreaseRequested() {
        boolean currentState = gamepadPrimary.dpad_up;
        boolean isPressed = currentState && !stateDpadUpPrev;
        stateDpadUpPrev = currentState;
        return isPressed;
    }

    /**
     * 检查D-Pad下是否被按下（用于减少转速）
     * 使用边沿检测：只在按下瞬间返回true
     * @return 本帧是否从未按下变为按下
     */
    public boolean isShooterPowerDecreaseRequested() {
        boolean currentState = gamepadPrimary.dpad_down;
        boolean isPressed = currentState && !stateDpadDownPrev;
        stateDpadDownPrev = currentState;
        return isPressed;
    }

    /**
     * 检查D-Pad左是否被按下（用于减少转速，步长较小）
     * 使用边沿检测：只在按下瞬间返回true
     * @return 本帧是否从未按下变为按下
     */
    public boolean isShooterPowerLeftRequested() {
        boolean currentState = gamepadPrimary.dpad_left;
        boolean isPressed = currentState && !stateDpadLeftPrev;
        stateDpadLeftPrev = currentState;
        return isPressed;
    }

    /**
     * 检查D-Pad右是否被按下（用于增加转速，步长较小）
     * 使用边沿检测：只在按下瞬间返回true
     * @return 本帧是否从未按下变为按下
     */
    public boolean isShooterPowerRightRequested() {
        boolean currentState = gamepadPrimary.dpad_right;
        boolean isPressed = currentState && !stateDpadRightPrev;
        stateDpadRightPrev = currentState;
        return isPressed;
    }

    // ==================== 控制模式切换输入 ====================

    /**
     * 检查Y键是否被按下（用于切换映射模式）
     * 使用边沿检测：只在按下瞬间返回true
     * @return 本帧是否从未按下变为按下
     */
    public boolean isMappingModeToggleRequested() {
        boolean currentState = gamepadPrimary.y;
        boolean isPressed = currentState && !stateYButtonPrev;
        stateYButtonPrev = currentState;
        return isPressed;
    }

    // ==================== 状态更新 ====================

    /**
     * 更新所有按键状态（必须在主循环中每帧调用）
     * 用于边沿检测的状态维护
     */
    public void updateButtonStates() {
        // 边沿检测状态会在各个getter方法中自动更新
        // 该方法为预留接口，便于将来扩展
    }

    // ==================== 调试方法 ====================

    /**
     * 获取当前控制输入状态字符串
     */
    public String getInputStatusString() {
        String status = "Chassis: FB=" + String.format("%.2f", getChassisDriveFBInput()) +
                       ", LR=" + String.format("%.2f", getChassisStrafeLRInput()) +
                       ", Rotate=" + String.format("%.2f", getChassisRotateCWInput()) +
                       " | Mode=" + (isChassisMappingNonlinear ? "NonLinear" : "Linear") +
                       " | Intake: A=" + isIntakeForwardRequested() + ", B=" + isIntakeReverseRequested() +
                       " | Load: LT=" + String.format("%.2f", getLoadTriggerValue()) +
                       " | Shooter: RT=" + String.format("%.2f", getShooterTriggerValue());
        return status;
    }
}
