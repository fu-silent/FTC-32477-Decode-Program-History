package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * 导航系统 v2.2.0 - IMU 和自动转向控制
 * 
 * 功能：
 * - 初始化 IMU 传感器
 * - 计算当前机器人航向
 * - 自动转向到目标角度（PID控制）
 * - 重置 IMU 偏航角
 */
public class NavigationSystem_2_2 {
    
    private final IMU imu;
    private final String imuName;
    
    // 自动转向状态
    private boolean isTurningToTarget = false;
    private double targetHeading = 0;
    
    // PID 控制状态
    private double integralSum = 0;
    private double previousError = 0;
    private long previousTime = 0;
    
    // 配置参数（从常数获取）
    private final double HEADING_THRESHOLD = RobotConstants_2_2.AUTO_TURN_HEADING_THRESHOLD;
    private final double TURN_POWER = RobotConstants_2_2.AUTO_TURN_POWER;
    private final double P_GAIN = RobotConstants_2_2.AUTO_TURN_P_GAIN;
    private final double I_GAIN = RobotConstants_2_2.AUTO_TURN_I_GAIN;
    private final double D_GAIN = RobotConstants_2_2.AUTO_TURN_D_GAIN;
    
    /**
     * 构造函数
     * @param imu IMU 硬件对象
     */
    public NavigationSystem_2_2(IMU imu, String imuName) {
        this.imu = imu;
        this.imuName = imuName;
    }
    
    /**
     * 初始化 IMU 传感器
     */
    public void initialize() {
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }
    
    /**
     * 重置 IMU 偏航角
     */
    public void resetHeading() {
        imu.resetYaw();
    }
    
    /**
     * 获取当前机器人航向（偏航角）
     * @return 当前航向角度（度），范围 -180 到 180
     */
    public double getCurrentHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    
    /**
     * 开始自动转向到目标角度
     * @param targetHeading 目标角度（度）
     */
    public void startAutoTurn(double targetHeading) {
        this.targetHeading = targetHeading;
        this.isTurningToTarget = true;
        resetPID();
    }
    
    /**
     * 检查是否正在自动转向
     * @return true 如果正在转向，false 否则
     */
    public boolean isAutoTurning() {
        return isTurningToTarget;
    }
    
    /**
     * 计算自动转向所需的旋转功率
     * 
     * @return 旋转功率（-1 到 1），如果转向完成则返回 0 并设置 isTurningToTarget = false
     */
    public double calculateAutoTurnPower() {
        if (!isTurningToTarget) {
            return 0;
        }
        
        double currentHeading = getCurrentHeading();
        double headingError = normalizeAngle(currentHeading - targetHeading);
        
        // 检查是否到达目标角度
        if (Math.abs(headingError) <= HEADING_THRESHOLD) {
            isTurningToTarget = false;
            return 0;
        }
        
        // 计算 PID 输出
        double turnPower = calculatePIDOutput(headingError);
        
        return turnPower;
    }
    
    /**
     * 计算 PID 控制输出
     * @param error 当前误差
     * @return PID 输出
     */
    private double calculatePIDOutput(double error) {
        long currentTime = System.nanoTime();
        double deltaTime = (currentTime - previousTime) / 1e9; // 转换为秒
        
        // 防止除零错误
        if (deltaTime == 0) {
            deltaTime = 0.01; // 默认 10ms
        }
        
        // 比例项
        double proportional = P_GAIN * error;
        
        // 积分项（带积分限幅）
        integralSum += error * deltaTime;
        double maxIntegral = 0.5 / Math.max(I_GAIN, 0.001);
        integralSum = Range.clip(integralSum, -maxIntegral, maxIntegral);
        double integral = I_GAIN * integralSum;
        
        // 微分项
        double derivative = D_GAIN * (error - previousError) / deltaTime;
        
        // 计算总输出
        double output = proportional + integral + derivative;
        
        // 限制输出范围
        output = Range.clip(output, -TURN_POWER, TURN_POWER);
        
        // 更新状态
        previousError = error;
        previousTime = currentTime;
        
        return output;
    }
    
    /**
     * 重置 PID 控制器
     */
    private void resetPID() {
        previousError = 0;
        integralSum = 0;
        previousTime = System.nanoTime();
    }
    
    /**
     * 标准化角度到 -180 到 180 度范围
     * @param angle 原始角度
     * @return 标准化后的角度
     */
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
    
    /**
     * 停止自动转向
     */
    public void stopAutoTurn() {
        isTurningToTarget = false;
    }
}
