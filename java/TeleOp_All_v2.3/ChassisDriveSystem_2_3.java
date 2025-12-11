package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * 底盘驱动系统 v2.3.0 - Mecanum 轮驱动和运动控制
 * 
 * 新增功能：
 * - 无头模式 (Field-Centric) 切换
 */
public class ChassisDriveSystem_2_3 {
    
    private final DcMotor motorChassisFL;  // 前左
    private final DcMotor motorChassisFR;  // 前右
    private final DcMotor motorChassisBL;  // 后左
    private final DcMotor motorChassisBR;  // 后右
    
    // 依赖
    private final NavigationSystem_2_3 navigation;

    // 控制参数
    private final double DEADZONE = RobotConstants_2_3.CHASSIS_JOYSTICK_DEADZONE;
    
    // 驱动模式状态
    private boolean isNonLinearMode = false;
    private boolean isFieldCentric = false;
    
    /**
     * 构造函数
     */
    public ChassisDriveSystem_2_3(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, NavigationSystem_2_3 nav) {
        this.motorChassisFL = fl;
        this.motorChassisFR = fr;
        this.motorChassisBL = bl;
        this.motorChassisBR = br;
        this.navigation = nav;
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
     * 切换驱动模式（线性 <-> 非线性）
     */
    public void toggleDriveMode() {
        isNonLinearMode = !isNonLinearMode;
    }

    /**
     * 切换驱动模式（有头 <-> 无头）
     */
    public void toggleCentricMode() {
        isFieldCentric = !isFieldCentric;
    }
    
    /**
     * 获取当前驱动模式名称
     */
    public String getDriveModeName() {
        String centric = isFieldCentric ? "无头 (Field)" : "有头 (Robot)";
        String linear = isNonLinearMode ? "非线性" : "线性";
        return centric + " | " + linear;
    }
    
    /**
     * 更新底盘运动
     */
    public void update(double drive, double strafe, double turn, 
                      boolean isAutoTurning, double autoTurnPower) {
        
        // 死区处理
        drive = applyDeadzone(drive);
        strafe = applyDeadzone(strafe);
        
        // 转向输入处理
        if (isAutoTurning) {
            turn = autoTurnPower;
        } else {
            turn = applyDeadzone(turn);
            if (isNonLinearMode) {
                drive = Math.copySign(drive * drive, drive);
                strafe = Math.copySign(strafe * strafe, strafe);
                turn = Math.copySign(turn * turn, turn);
            }
        }

        // 无头模式坐标转换
        if (isFieldCentric) {
            double botHeading = Math.toRadians(navigation.getHeading());
            double rotX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
            double rotY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);
            strafe = rotX;
            drive = rotY;
        }
        
        // 麦克纳姆轮解算
        double flPower = drive + strafe + turn;
        double frPower = drive - strafe - turn;
        double blPower = drive - strafe + turn;
        double brPower = drive + strafe - turn;
        
        // 功率归一化
        normalizeAndApplyPower(flPower, frPower, blPower, brPower);
    }
    
    /**
     * 应用死区
     */
    private double applyDeadzone(double input) {
        if (Math.abs(input) < DEADZONE) {
            return 0;
        }
        return input;
    }
    
    /**
     * 归一化功率并应用到电机
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
