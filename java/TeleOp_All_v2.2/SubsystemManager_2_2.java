package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * 子系统管理器 v2.2.0 - 拾取、装填、双发射系统
 * 
 * 新增功能：
 * - 双发射电机独立 PIDF 控制
 * - 多转速档位预设
 * - 转速精度自适应
 * - 发射状态控制（扳机触发）
 * - 获取各子系统状态字符串
 */
public class SubsystemManager_2_2 {
    
    private final DcMotor motorIntake;
    private final DcMotor motorLoad;
    private final DcMotorEx motorShooter1;
    private final DcMotorEx motorShooter2;
    
    // 转速和精度设置
    private int targetRPM = 0;
    private int currentErrorRange = 50;
    private boolean isShooting = false;
    
    // 状态机 (仅用于显示)
    private int fireState = RobotConstants_2_2.STATE_IDLE;
    
    // 编码器常数
    private static final double MOTOR_TICK_COUNT = RobotConstants_2_2.SHOOTER_MOTOR_TICK_COUNT;
    
    /**
     * 构造函数
     */
    public SubsystemManager_2_2(DcMotor intake, DcMotor load, 
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
            RobotConstants_2_2.SHOOTER_PIDF_P,
            RobotConstants_2_2.SHOOTER_PIDF_I,
            RobotConstants_2_2.SHOOTER_PIDF_D,
            RobotConstants_2_2.SHOOTER_PIDF_F
        );
        
        motorShooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        motorShooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }
    
    /**
     * 启动拾取（正向）
     */
    public void intakeStart() {
        motorIntake.setPower(RobotConstants_2_2.INTAKE_FORWARD_POWER);
    }
    
    /**
     * 反向拾取
     */
    public void intakeReverse() {
        motorIntake.setPower(RobotConstants_2_2.INTAKE_REVERSE_POWER);
    }
    
    /**
     * 停止拾取
     */
    public void intakeStop() {
        motorIntake.setPower(RobotConstants_2_2.INTAKE_STOP_POWER);
    }
    
    /**
     * 启动装填
     */
    public void loadStart() {
        motorLoad.setPower(RobotConstants_2_2.LOAD_FORWARD_POWER);
    }
    
    /**
     * 反向装填
     */
    public void loadReverse() {
        motorLoad.setPower(RobotConstants_2_2.LOAD_REVERSE_POWER);
    }
    
    /**
     * 停止装填
     */
    public void loadStop() {
        motorLoad.setPower(RobotConstants_2_2.LOAD_STOP_POWER);
    }
    
    /**
     * 设置发射转速（RPM）并自动调整精度范围
     * @param rpm 目标转速（RPM）
     */
    public void setShooterTargetRPM(int rpm) {
        this.targetRPM = rpm;
        
        // 根据转速档位自动调整精度范围
        if (rpm == RobotConstants_2_2.SHOOTER_RPM_LONG_RANGE) {
            this.currentErrorRange = RobotConstants_2_2.SHOOTER_RPM_ERROR_RANGE_LONG;
        } else if (rpm == RobotConstants_2_2.SHOOTER_RPM_TRIANGLE_SIDE) {
            this.currentErrorRange = RobotConstants_2_2.SHOOTER_RPM_ERROR_RANGE_SIDE;
        } else if (rpm == RobotConstants_2_2.SHOOTER_RPM_TRIANGLE_BASE) {
            this.currentErrorRange = RobotConstants_2_2.SHOOTER_RPM_ERROR_RANGE_BASE;
        } else if (rpm == RobotConstants_2_2.SHOOTER_RPM_TRIANGLE_TOP) {
            this.currentErrorRange = RobotConstants_2_2.SHOOTER_RPM_ERROR_RANGE_TOP;
        }
        
        // 如果正在发射中，更新转速
        if (isShooting) {
            updateShooterSpeed();
        }
    }
    
    /**
     * 设置发射状态（启/停）
     */
    public void setShootingState(boolean firing) {
        if (isShooting != firing) {
            isShooting = firing;
            updateShooterSpeed();
            
            // 更新显示用的状态
            fireState = firing ? RobotConstants_2_2.STATE_FIRING : RobotConstants_2_2.STATE_IDLE;
        }
    }

    /**
     * 更新发射电机速度
     */
    private void updateShooterSpeed() {
        double ticks = 0;
        if (isShooting) {
            ticks = targetRPM * MOTOR_TICK_COUNT / 60.0;
        }
        motorShooter1.setVelocity(ticks);
        motorShooter2.setVelocity(ticks);
    }
    
    /**
     * 获取当前发射转速（从第二个发射电机读取）- 兼容旧接口
     * @return 当前转速（RPM）
     */
    public double getCurrentShooterRPM() {
        return getCurrentShooter2RPM();
    }
    
    /**
     * 获取S1电机转速
     */
    public double getCurrentShooter1RPM() {
        return (motorShooter1.getVelocity() / MOTOR_TICK_COUNT) * 60.0;
    }

    /**
     * 获取S2电机转速
     */
    public double getCurrentShooter2RPM() {
        return (motorShooter2.getVelocity() / MOTOR_TICK_COUNT) * 60.0;
    }
    
    /**
     * 检查发射转速是否达到目标（在精度范围内）
     * @return true 如果达到目标转速，false 否则
     */
    public boolean isShooterAtTargetSpeed() {
        if (!isShooting || targetRPM == 0) return false;
        double currentRPM = getCurrentShooter2RPM();
        double targetMin = targetRPM - currentErrorRange;
        double targetMax = targetRPM + currentErrorRange;
        return currentRPM >= targetMin && currentRPM <= targetMax;
    }
    
    /**
     * 停止发射电机
     */
    public void stopShooter() {
        isShooting = false;
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
    
    /**
     * 获取拾取状态字符串
     */
    public String getIntakeStatus() {
        double p = motorIntake.getPower();
        if (p > 0) return "吸入";
        if (p < 0) return "吐出";
        return "停止";
    }

    /**
     * 获取装填状态字符串
     */
    public String getLoadStatus() {
        double p = motorLoad.getPower();
        if (p > 0) return "装填";
        if (p < 0) return "退回";
        return "停止";
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
