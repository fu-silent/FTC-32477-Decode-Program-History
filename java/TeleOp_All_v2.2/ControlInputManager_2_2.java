package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * 输入管理器 v2.2.0 - 手柄输入处理
 * 
 * 功能：
 * - 摇杆输入处理
 * - 按键边缘检测
 * - D-Pad 转速档位控制
 * - 肩键自动转向控制
 * - 手柄震动反馈
 */
public class ControlInputManager_2_2 {
    
    private final Gamepad gamepad;
    
    // 边缘检测用的状态
    private boolean lastYState = false;
    private boolean lastRightBumperState = false;
    private boolean lastUp = false, lastDown = false, lastLeft = false, lastRight = false;
    
    public ControlInputManager_2_2(Gamepad gamepad) {
        this.gamepad = gamepad;
    }
    
    /**
     * 获取前后运动输入（左摇杆 Y 轴）
     * @return -1 到 1，正向前进
     */
    public double getChassisDriveFBInput() {
        return -gamepad.left_stick_y;  // 向上推为负值，需要反转
    }
    
    /**
     * 获取左右平移输入（左摇杆 X 轴）
     * @return -1 到 1，正向右移
     */
    public double getChassisStrafeLRInput() {
        return gamepad.left_stick_x;
    }
    
    /**
     * 获取旋转输入（右摇杆 X 轴）
     * @return -1 到 1，正向逆时针
     */
    public double getChassisRotateCWInput() {
        return gamepad.right_stick_x;
    }
    
    // ========== 拾取和装填控制 ==========
    
    /**
     * 检查是否按下 A 键（拾取进）
     */
    public boolean isIntakeForwardRequested() {
        return gamepad.a;
    }
    
    /**
     * 检查是否按下 B 键（拾取反）
     */
    public boolean isIntakeReverseRequested() {
        return gamepad.b;
    }
    
    /**
     * 检查左扳机是否被按下（装填系统）
     */
    public boolean isLoadRequested() {
        return gamepad.left_trigger > 0.1;
    }
    
    /**
     * 检查左肩键是否被按下（装填反转）
     */
    public boolean isLoadReverseRequested() {
        return gamepad.left_bumper;
    }
    
    /**
     * 检查是否按下 X 键（停止所有）
     */
    public boolean isStopRequested() {
        return gamepad.x;
    }
    
    // ========== 发射控制 ==========
    
    /**
     * 检查右扳机是否被按下（发射系统）
     */
    public boolean isShooterRequested() {
        return gamepad.right_trigger > 0.1;
    }
    
    /**
     * 检查是否按下 D-Pad 右（超远距离转速）- 边缘触发
     */
    public boolean isShooterSpeedLongRangeRequested() {
        boolean current = gamepad.dpad_right;
        boolean result = current && !lastRight;
        lastRight = current;
        return result;
    }
    
    /**
     * 检查是否按下 D-Pad 左（三角腰转速）- 边缘触发
     */
    public boolean isShooterSpeedSideRequested() {
        boolean current = gamepad.dpad_left;
        boolean result = current && !lastLeft;
        lastLeft = current;
        return result;
    }
    
    /**
     * 检查是否按下 D-Pad 下（三角底转速）- 边缘触发
     */
    public boolean isShooterSpeedBaseRequested() {
        boolean current = gamepad.dpad_down;
        boolean result = current && !lastDown;
        lastDown = current;
        return result;
    }
    
    /**
     * 检查是否按下 D-Pad 上（三角顶转速）- 边缘触发
     */
    public boolean isShooterSpeedTopRequested() {
        boolean current = gamepad.dpad_up;
        boolean result = current && !lastUp;
        lastUp = current;
        return result;
    }
    
    /**
     * 触发手柄震动
     */
    public void rumble() {
        gamepad.rumble(RobotConstants_2_2.RUMBLE_DURATION_MS);
    }
    
    /**
     * 检查 Y 键是否被按下（边缘触发）
     * 
     * @return true 如果 Y 键从未按下变为按下
     */
    public boolean isYButtonPressed() {
        boolean currentYState = gamepad.y;
        boolean result = currentYState && !lastYState;
        lastYState = currentYState;
        return result;
    }
    
    /**
     * 获取 Y 键当前状态（用于手动状态跟踪）
     */
    public boolean getYButtonState() {
        return gamepad.y;
    }
    
    /**
     * 更新 Y 键上一状态（在主循环中调用）
     */
    public void updateYButtonState() {
        lastYState = gamepad.y;
    }
    
    // ========== 自动转向控制 ==========
    
    /**
     * 检查是否按下右肩键（自动转向 45 度）
     * @return true 如果右肩键从未按下变为按下
     */
    public boolean isRightBumperPressed() {
        boolean currentRightBumperState = gamepad.right_bumper;
        boolean result = currentRightBumperState && !lastRightBumperState;
        lastRightBumperState = currentRightBumperState;
        return result;
    }
    
    /**
     * 获取右肩键当前状态（用于手动状态跟踪）
     */
    public boolean getRightBumperState() {
        return gamepad.right_bumper;
    }
    
    /**
     * 更新右肩键上一状态（在主循环中调用）
     */
    public void updateRightBumperState() {
        lastRightBumperState = gamepad.right_bumper;
    }
    
    /**
     * 检查是否按下左肩键
     */
    public boolean isLeftBumperPressed() {
        return gamepad.left_bumper;
    }
}
