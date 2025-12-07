package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * 子系统管理器 v2.0.0 - 拾取、装填、双发射系统
 * 
 * 新增功能：
 * - 双发射电机独立 PIDF 控制
 * - 多转速档位预设
 * - 转速精度自适应
 * - 状态机控制发射流程
 */
public class SubsystemManager_2_0 {
    
    private final DcMotor motorIntake;
    private final DcMotor motorLoad;
    private final DcMotorEx motorShooter1;
    private final DcMotorEx motorShooter2;
    
    // 转速和精度设置
    private int targetRPM = 0;
    private int currentErrorRange = 50;
    
    // 状态机
    private int fireState = RobotConstants_2_0.STATE_IDLE;
    
    // 编码器常数
    private static final double MOTOR_TICK_COUNT = RobotConstants_2_0.SHOOTER_MOTOR_TICK_COUNT;
    
    /**
     * 构造函数
     */
    public SubsystemManager_2_0(DcMotor intake, DcMotor load, 
                                DcMotorEx shooter1, DcMotorEx shooter2) {
        this.motorIntake = intake;
        this.motorLoad = load;
        this.motorShooter1 = shooter1;
        this.motorShooter2 = shooter2;
    }
    
    /**
     * 初始化所有子系统电机
     */
    public void initialize() {
        // 设置方向
        motorIntake.setDirection(DcMotor.Direction.FORWARD);
        motorLoad.setDirection(DcMotor.Direction.REVERSE);
        motorShooter1.setDirection(DcMotor.Direction.FORWARD);
        motorShooter2.setDirection(DcMotor.Direction.REVERSE);
        
        // 设置断电行为
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLoad.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorShooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorShooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // 配置发射电机编码器和 PIDF
        motorShooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorShooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(
            RobotConstants_2_0.SHOOTER_PIDF_P,
            RobotConstants_2_0.SHOOTER_PIDF_I,
            RobotConstants_2_0.SHOOTER_PIDF_D,
            RobotConstants_2_0.SHOOTER_PIDF_F
        );
        
        motorShooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        motorShooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }
    
    /**
     * 启动拾取（正向）
     */
    public void intakeStart() {
        motorIntake.setPower(RobotConstants_2_0.INTAKE_FORWARD_POWER);
    }
    
    /**
     * 反向拾取
     */
    public void intakeReverse() {
        motorIntake.setPower(RobotConstants_2_0.INTAKE_REVERSE_POWER);
    }
    
    /**
     * 停止拾取
     */
    public void intakeStop() {
        motorIntake.setPower(RobotConstants_2_0.INTAKE_STOP_POWER);
    }
    
    /**
     * 启动装填
     */
    public void loadStart() {
        motorLoad.setPower(RobotConstants_2_0.LOAD_FORWARD_POWER);
    }
    
    /**
     * 反向装填
     */
    public void loadReverse() {
        motorLoad.setPower(RobotConstants_2_0.LOAD_REVERSE_POWER);
    }
    
    /**
     * 停止装填
     */
    public void loadStop() {
        motorLoad.setPower(RobotConstants_2_0.LOAD_STOP_POWER);
    }
    
    /**
     * 设置发射转速（RPM）并自动调整精度范围
     * @param rpm 目标转速（RPM）
     */
    public void setShooterTargetRPM(int rpm) {
        this.targetRPM = rpm;
        
        // 根据转速档位自动调整精度范围
        if (rpm == RobotConstants_2_0.SHOOTER_RPM_LONG_RANGE) {
            this.currentErrorRange = RobotConstants_2_0.SHOOTER_RPM_ERROR_RANGE_LONG;
        } else if (rpm == RobotConstants_2_0.SHOOTER_RPM_TRIANGLE_SIDE) {
            this.currentErrorRange = RobotConstants_2_0.SHOOTER_RPM_ERROR_RANGE_SIDE;
        } else if (rpm == RobotConstants_2_0.SHOOTER_RPM_TRIANGLE_BASE) {
            this.currentErrorRange = RobotConstants_2_0.SHOOTER_RPM_ERROR_RANGE_BASE;
        } else if (rpm == RobotConstants_2_0.SHOOTER_RPM_TRIANGLE_TOP) {
            this.currentErrorRange = RobotConstants_2_0.SHOOTER_RPM_ERROR_RANGE_TOP;
        }
        
        // 更新电机目标速度
        updateShooterSpeed();
    }
    
    /**
     * 更新发射电机速度
     */
    private void updateShooterSpeed() {
        double targetTicksPerSecond = targetRPM * MOTOR_TICK_COUNT / 60.0;
        motorShooter1.setVelocity(targetTicksPerSecond);
        motorShooter2.setVelocity(targetTicksPerSecond);
    }
    
    /**
     * 获取当前发射转速（从第二个发射电机读取）
     * @return 当前转速（RPM）
     */
    public double getCurrentShooterRPM() {
        double currentVelocityTicks = motorShooter2.getVelocity();
        return (currentVelocityTicks / MOTOR_TICK_COUNT) * 60.0;
    }
    
    /**
     * 检查发射转速是否达到目标（在精度范围内）
     * @return true 如果达到目标转速，false 否则
     */
    public boolean isShooterAtTargetSpeed() {
        double currentRPM = getCurrentShooterRPM();
        double targetMin = targetRPM - currentErrorRange;
        double targetMax = targetRPM + currentErrorRange;
        return currentRPM >= targetMin && currentRPM <= targetMax;
    }
    
    /**
     * 停止发射电机
     */
    public void stopShooter() {
        targetRPM = 0;
        updateShooterSpeed();
    }
    
    /**
     * 停止所有子系统
     */
    public void stopAll() {
        intakeStop();
        loadStop();
        stopShooter();
    }
    
    // ========== 状态机控制 ==========
    
    /**
     * 更新发射状态机
     * 
     * A键：启动拾取
     * B键：反向拾取和装填
     * X键：停止所有
     * Y键（边缘触发）：根据转速准备情况自动控制发射流程
     */
    public void updateFireState(boolean yPressed, boolean lastYState) {
        // Y 键边缘检测
        if (yPressed && !lastYState) {
            if (fireState == RobotConstants_2_0.STATE_IDLE && isShooterAtTargetSpeed()) {
                // 从待机 -> 发射状态
                fireState = RobotConstants_2_0.STATE_FIRING;
                intakeStart();
                loadStart();
            } else if (fireState == RobotConstants_2_0.STATE_FIRING) {
                // 从发射状态 -> 待机
                fireState = RobotConstants_2_0.STATE_IDLE;
                intakeStop();
                loadStop();
            }
        }
        
        // 如果转速不足，停止发射
        if (fireState == RobotConstants_2_0.STATE_FIRING && !isShooterAtTargetSpeed()) {
            intakeStop();
            loadStop();
        }
    }
    
    /**
     * 获取当前火控状态
     */
    public int getFireState() {
        return fireState;
    }
    
    /**
     * 获取目标转速
     */
    public int getTargetRPM() {
        return targetRPM;
    }
    
    /**
     * 获取精度范围
     */
    public int getCurrentErrorRange() {
        return currentErrorRange;
    }
}
