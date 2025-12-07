package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * 底盘驱动系统 v2.1.0 - Mecanum 轮驱动和运动控制
 * 
 * 功能：
 * - 前后、左右、旋转独立控制
 * - 自动转向时禁用手动转向
 * - 死区和非线性映射
 * - 功率归一化
 */
public class ChassisDriveSystem_2_1 {
    
    private final DcMotor motorChassisFL;  // 前左
    private final DcMotor motorChassisFR;  // 前右
    private final DcMotor motorChassisBL;  // 后左
    private final DcMotor motorChassisBR;  // 后右
    
    // 控制参数
    private final double DEADZONE = RobotConstants_2_1.CHASSIS_JOYSTICK_DEADZONE;
    
    /**
     * 构造函数
     */
    public ChassisDriveSystem_2_1(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {
        this.motorChassisFL = fl;
        this.motorChassisFR = fr;
        this.motorChassisBL = bl;
        this.motorChassisBR = br;
    }
    
    /**
     * 初始化底盘电机
     */
    public void initialize() {
        // 设置电机方向
        motorChassisFL.setDirection(DcMotor.Direction.FORWARD);
        motorChassisFR.setDirection(DcMotor.Direction.REVERSE);
        motorChassisBL.setDirection(DcMotor.Direction.REVERSE);
        motorChassisBR.setDirection(DcMotor.Direction.FORWARD);
        
        // 设置断电行为为制动
        motorChassisFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorChassisFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorChassisBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorChassisBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    /**
     * 更新底盘运动
     * 
     * @param drive 前后运动（-1 到 1，正向前进）
     * @param strafe 左右运动（-1 到 1，正向右移）
     * @param turn 旋转运动（-1 到 1，正向逆时针）
     * @param isAutoTurning 是否正在自动转向（如果是，忽略手动 turn 输入）
     * @param autoTurnPower 自动转向功率（仅在 isAutoTurning 为 true 时使用）
     */
    public void update(double drive, double strafe, double turn, 
                      boolean isAutoTurning, double autoTurnPower) {
        
        // 应用死区
        drive = applyDeadzone(drive);
        strafe = applyDeadzone(strafe);
        
        // 如果正在自动转向，使用自动转向功率替代手动转向
        if (isAutoTurning) {
            turn = autoTurnPower;
        } else {
            turn = applyDeadzone(turn);
            turn = applyNonlinearMapping(turn);
        }
        
        // 计算各轮功率（Mecanum 轮运动学）
        double flPower = drive + strafe + turn;
        double frPower = drive - strafe - turn;
        double blPower = drive + strafe - turn;
        double brPower = drive - strafe + turn;
        
        // 功率归一化
        normalizeAndApplyPower(flPower, frPower, blPower, brPower);
    }
    
    /**
     * 应用死区（摇杆小于此值时输出 0）
     * @param input 输入值（-1 到 1）
     * @return 应用死区后的值
     */
    private double applyDeadzone(double input) {
        if (Math.abs(input) < DEADZONE) {
            return 0;
        }
        return input;
    }
    
    /**
     * 应用非线性映射（平方映射，增加精细控制）
     * @param input 输入值（-1 到 1）
     * @return 映射后的值
     */
    private double applyNonlinearMapping(double input) {
        // 保留符号，对绝对值进行平方
        return Math.copySign(input * input, input);
    }
    
    /**
     * 归一化功率并应用到电机
     * 
     * 如果任何轮的功率超过 ±1.0，则所有轮都按相同比例缩放
     */
    private void normalizeAndApplyPower(double fl, double fr, double bl, double br) {
        double maxPower = Math.max(
            Math.max(Math.abs(fl), Math.abs(fr)),
            Math.max(Math.abs(bl), Math.abs(br))
        );
        
        if (maxPower > 1.0) {
            fl /= maxPower;
            fr /= maxPower;
            bl /= maxPower;
            br /= maxPower;
        }
        
        motorChassisFL.setPower(fl);
        motorChassisFR.setPower(fr);
        motorChassisBL.setPower(bl);
        motorChassisBR.setPower(br);
    }
    
    /**
     * 停止所有底盘电机
     */
    public void stop() {
        motorChassisFL.setPower(0);
        motorChassisFR.setPower(0);
        motorChassisBL.setPower(0);
        motorChassisBR.setPower(0);
    }
}
