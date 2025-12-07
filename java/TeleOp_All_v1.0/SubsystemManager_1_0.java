package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * 子系统管理类
 * 负责机器人的三个独立子系统：拾取(Intake)、装填(Load)、发射(Shooter)
 * 职责：
 * - 初始化各子系统的电机
 * - 提供统一的子系统控制接口
 * - 管理发射模块的PID参数和速度反馈
 */
public class SubsystemManager_1_0 {

    // ==================== 硬件对象 ====================
    
    // 拾取系统
    private DcMotor motorSubsystemIntake;
    
    // 装填系统
    private DcMotor motorSubsystemLoad;
    
    // 发射系统（双电机）
    private DcMotorEx motorSubsystemShooterS1;
    private DcMotorEx motorSubsystemShooterS2;

    // ==================== 发射模块状态变量 ====================
    private double shooterPower;
    private boolean isInitialized = false;

    // ==================== 构造函数 ====================
    public SubsystemManager_1_0() {
        shooterPower = RobotConstants_1_0.SHOOTER_POWER_DEFAULT;
    }

    // ==================== 初始化方法 ====================

    /**
     * 从HardwareMap初始化所有子系统电机
     * @param hardwareMap FTC硬件映射对象
     * @return 初始化是否成功
     */
    public boolean initialize(HardwareMap hardwareMap) {
        try {
            // 初始化拾取系统
            motorSubsystemIntake = hardwareMap.get(DcMotor.class, RobotConstants_1_0.SUBSYSTEM_MOTOR_INTAKE_NAME);
            motorSubsystemIntake.setDirection(DcMotor.Direction.FORWARD);
            motorSubsystemIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // 初始化装填系统
            motorSubsystemLoad = hardwareMap.get(DcMotor.class, RobotConstants_1_0.SUBSYSTEM_MOTOR_LOAD_NAME);
            motorSubsystemLoad.setDirection(DcMotor.Direction.REVERSE);
            motorSubsystemLoad.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // 初始化发射系统（S1、S2）
            motorSubsystemShooterS1 = hardwareMap.get(DcMotorEx.class, RobotConstants_1_0.SUBSYSTEM_MOTOR_SHOOTER_S1_NAME);
            motorSubsystemShooterS2 = hardwareMap.get(DcMotorEx.class, RobotConstants_1_0.SUBSYSTEM_MOTOR_SHOOTER_S2_NAME);

            motorSubsystemShooterS1.setDirection(DcMotor.Direction.FORWARD);
            motorSubsystemShooterS2.setDirection(DcMotor.Direction.FORWARD);

            motorSubsystemShooterS1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorSubsystemShooterS2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // 设置发射电机零功率行为为制动（防止怠速旋转）
            motorSubsystemShooterS1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorSubsystemShooterS2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // 停止所有子系统电机
            stopAll();

            isInitialized = true;
            return true;

        } catch (Exception e) {
            isInitialized = false;
            return false;
        }
    }

    /**
     * 检查是否初始化成功
     */
    public boolean isInitialized() {
        return isInitialized;
    }

    // ==================== 拾取系统(Intake)控制 ====================

    /**
     * 启动拾取电机正向转动
     */
    public void intakeStart() {
        motorSubsystemIntake.setPower(RobotConstants_1_0.SUBSYSTEM_INTAKE_MOTOR_POWER);
    }

    /**
     * 启动拾取电机反向转动
     */
    public void intakeReverse() {
        motorSubsystemIntake.setPower(RobotConstants_1_0.SUBSYSTEM_INTAKE_REVERSE_POWER);
    }

    /**
     * 停止拾取电机
     */
    public void intakeStop() {
        motorSubsystemIntake.setPower(0.0);
    }

    /**
     * 获取拾取电机当前功率
     */
    public double getIntakePower() {
        return motorSubsystemIntake.getPower();
    }

    // ==================== 装填系统(Load)控制 ====================

    /**
     * 启动装填电机
     */
    public void loadStart() {
        motorSubsystemLoad.setPower(RobotConstants_1_0.SUBSYSTEM_LOAD_MOTOR_POWER);
    }

    /**
     * 启动装填电机反向
     */
    public void loadReverse() {
        motorSubsystemLoad.setPower(-RobotConstants_1_0.SUBSYSTEM_LOAD_MOTOR_POWER);
    }

    /**
     * 停止装填电机
     */
    public void loadStop() {
        motorSubsystemLoad.setPower(0.0);
    }

    /**
     * 获取装填电机当前功率
     */
    public double getLoadPower() {
        return motorSubsystemLoad.getPower();
    }

    // ==================== 发射系统(Shooter)控制 ====================

    /**
     * 启动发射模块（S1正转，S2反转）
     */
    public void shooterStart() {
        // 使用功率控制，S1 与 S2 方向相反
        motorSubsystemShooterS1.setPower(shooterPower);
        motorSubsystemShooterS2.setPower(-shooterPower);
    }

    /**
     * 停止发射模块
     */
    public void shooterStop() {
        motorSubsystemShooterS1.setPower(0.0);
        motorSubsystemShooterS2.setPower(0.0);
    }

    /**
     * 设置发射功率（0.0 - 1.0）
     */
    public void setShooterPower(double power) {
        shooterPower = Math.max(RobotConstants_1_0.SHOOTER_POWER_MIN, Math.min(RobotConstants_1_0.SHOOTER_POWER_MAX, power));
    }

    /**
     * 增加发射功率（步长由常量控制）
     */
    public void increaseShooterPower() {
        setShooterPower(shooterPower + RobotConstants_1_0.SHOOTER_POWER_STEP);
    }

    /**
     * 减少发射功率（步长由常量控制）
     */
    public void decreaseShooterPower() {
        setShooterPower(shooterPower - RobotConstants_1_0.SHOOTER_POWER_STEP);
    }

    /**
     * 增加发射功率（较小步长，D-Pad左右）
     */
    public void increaseShooterPowerSmall() {
        setShooterPower(shooterPower + RobotConstants_1_0.SHOOTER_POWER_STEP_LR);
    }

    /**
     * 减少发射功率（较小步长，D-Pad左右）
     */
    public void decreaseShooterPowerSmall() {
        setShooterPower(shooterPower - RobotConstants_1_0.SHOOTER_POWER_STEP_LR);
    }

    /**
     * 获取当前发射功率
     */
    public double getShooterPower() {
        return shooterPower;
    }

    /**
     * 获取发射模块S1当前功率
     */
    public double getShooterS1Power() {
        return motorSubsystemShooterS1.getPower();
    }

    /**
     * 获取发射模块S2当前功率
     */
    public double getShooterS2Power() {
        return motorSubsystemShooterS2.getPower();
    }

    // ==================== 批量控制方法 ====================

    /**
     * 停止所有子系统电机
     */
    public void stopAll() {
        intakeStop();
        loadStop();
        shooterStop();
    }

    // ==================== 调试方法 ====================

    /**
     * 获取子系统初始化信息
     */
    public String getInitializationStatus() {
        if (!isInitialized) {
            return "子系统未初始化";
        }
        return "子系统已初始化: Intake=" + getIntakePower() + 
               ", Load=" + getLoadPower() + 
               ", Shooter(S1)=" + getShooterS1Power() + 
               ", Shooter(S2)=" + getShooterS2Power();
    }

    /**
     * 获取发射模块详细状态
     */
    public String getShooterStatus() {
        // 现在使用功率控制，返回当前功率信息
        return "Shooter Power: " + String.format("%.2f", shooterPower) +
               " | S1 Power: " + String.format("%.2f", getShooterS1Power()) +
               " | S2 Power: " + String.format("%.2f", getShooterS2Power());
    }
}
