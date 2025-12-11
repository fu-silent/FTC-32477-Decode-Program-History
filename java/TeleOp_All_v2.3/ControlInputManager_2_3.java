package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * 输入管理器 v2.3.0 - 手柄输入处理
 * 
 * 新增功能：
 * - X 键检测（切换有头/无头模式）
 * - Back/Options 键检测（重置 IMU）
 */
public class ControlInputManager_2_3 {
    
    private final Gamepad gamepad;
    
    // 边缘检测用的状态
    private boolean lastYState = false;
    private boolean lastXState = false;
    private boolean lastOptionsState = false;
    private boolean lastRightBumperState = false;
    private boolean lastUp = false, lastDown = false, lastLeft = false, lastRight = false;
    
    public ControlInputManager_2_3(Gamepad gamepad) {
        this.gamepad = gamepad;
    }
    
    /**
     * 获取前后运动输入（左摇杆 Y 轴）
     */
    public double getChassisDriveFBInput() {
        return -gamepad.left_stick_y;
    }
    
    /**
     * 获取左右平移输入（左摇杆 X 轴）
     */
    public double getChassisStrafeLRInput() {
        return gamepad.left_stick_x;
    }
    
    /**
     * 获取旋转输入（右摇杆 X 轴）
     */
    public double getChassisRotateCWInput() {
        return gamepad.right_stick_x;
    }
    
    // ========== 拾取和装填控制 ==========
    
    public boolean isIntakeForwardRequested() {
        return gamepad.a;
    }
    
    public boolean isIntakeReverseRequested() {
        return gamepad.b;
    }
    
    public boolean isLoadRequested() {
        return gamepad.left_trigger > 0.1;
    }
    
    public boolean isLoadReverseRequested() {
        return gamepad.left_bumper;
    }
    
    // ========== 发射控制 ==========
    
    public boolean isShooterRequested() {
        return gamepad.right_trigger > 0.1;
    }
    
    public boolean isShooterSpeedLongRangeRequested() {
        boolean current = gamepad.dpad_right;
        boolean result = current && !lastRight;
        lastRight = current;
        return result;
    }
    
    public boolean isShooterSpeedSideRequested() {
        boolean current = gamepad.dpad_left;
        boolean result = current && !lastLeft;
        lastLeft = current;
        return result;
    }
    
    public boolean isShooterSpeedBaseRequested() {
        boolean current = gamepad.dpad_down;
        boolean result = current && !lastDown;
        lastDown = current;
        return result;
    }
    
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
        gamepad.rumble(RobotConstants_2_3.RUMBLE_DURATION_MS);
    }
    
    /**
     * 检查 Y 键是否被按下（边缘触发）
     */
    public boolean isYButtonPressed() {
        boolean currentYState = gamepad.y;
        boolean result = currentYState && !lastYState;
        lastYState = currentYState;
        return result;
    }
    
    /**
     * 检查 X 键是否被按下（边缘触发）
     */
    public boolean isXButtonPressed() {
        boolean currentXState = gamepad.x;
        boolean result = currentXState && !lastXState;
        lastXState = currentXState;
        return result;
    }

    /**
     * 检查 Back/Options 键是否被按下（边缘触发）
     */
    public boolean isOptionsPressed() {
        boolean currentOptionsState = gamepad.back || gamepad.options;
        boolean result = currentOptionsState && !lastOptionsState;
        lastOptionsState = currentOptionsState;
        return result;
    }
    
    // ========== 自动转向控制 ==========
    
    /**
     * 检查是否按下右肩键
     */
    public boolean isRightBumperPressed() {
        boolean currentRightBumperState = gamepad.right_bumper;
        boolean result = currentRightBumperState && !lastRightBumperState;
        lastRightBumperState = currentRightBumperState;
        return result;
    }
}
