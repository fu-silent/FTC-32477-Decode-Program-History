package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * 遥测管理器 v2.1.0 - 系统状态显示
 * 
 * 显示内容：
 * - 底盘运动状态
 * - 发射系统状态和转速
 * - IMU 航向角
 * - 自动转向状态
 * - 按键提示
 */
public class TelemetryManager_2_1 {
    
    private final Telemetry telemetry;
    
    public TelemetryManager_2_1(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    
    /**
     * 显示运行时间和基本状态
     */
    public void displayRuntimeStatus(String runtimeString) {
        telemetry.addLine("========== TeleOp v2.1 ==========");
        telemetry.addData("运行时间", runtimeString);
        telemetry.addData("状态", "运行中");
    }
    
    /**
     * 显示发射系统状态
     */
    public void displayShooterStatus(int targetRPM, double currentRPM, 
                                    boolean isAtTargetSpeed, int fireState) {
        telemetry.addLine("--- 发射系统 ---");
        telemetry.addData("目标转速", "%d RPM", targetRPM);
        telemetry.addData("当前转速", "%.0f RPM", currentRPM);
        telemetry.addData("转速达标", isAtTargetSpeed ? "✓ 是" : "✗ 否");
        
        String fireStateStr;
        switch (fireState) {
            case RobotConstants_2_1.STATE_IDLE:
                fireStateStr = "待机";
                break;
            case RobotConstants_2_1.STATE_PREPARING:
                fireStateStr = "准备中";
                break;
            case RobotConstants_2_1.STATE_READY_TO_FIRE:
                fireStateStr = "准备好";
                break;
            case RobotConstants_2_1.STATE_FIRING:
                fireStateStr = "发射中";
                break;
            default:
                fireStateStr = "未知";
        }
        telemetry.addData("发射状态", fireStateStr);
    }
    
    /**
     * 显示 IMU 和自动转向状态
     */
    public void displayNavigationStatus(double currentHeading, double targetHeading, 
                                       boolean isAutoTurning) {
        telemetry.addLine("--- 导航系统 ---");
        telemetry.addData("当前航向", "%.1f°", currentHeading);
        telemetry.addData("目标航向", "%.1f°", targetHeading);
        telemetry.addData("自动转向", isAutoTurning ? "进行中" : "关闭");
    }
    
    /**
     * 显示按键提示和控制说明
     */
    public void displayControlHints() {
        telemetry.addLine("--- 控制说明 ---");
        telemetry.addLine("左摇杆：前后/左右移动");
        telemetry.addLine("右摇杆：旋转");
        telemetry.addLine("A键：拾取进  B键：拾取退");
        telemetry.addLine("X键：停止  Y键：发射/停止");
        telemetry.addLine("D-Pad右：超远  D-Pad左：腰部");
        telemetry.addLine("D-Pad下：底部  D-Pad上：顶点");
        telemetry.addLine("右肩键：自动转向45°");
    }
    
    /**
     * 显示完整的遥测信息（综合显示）
     */
    public void displayFullTelemetry(
            String runtimeString,
            int targetRPM,
            double currentRPM,
            boolean isAtTargetSpeed,
            int fireState,
            double currentHeading,
            double targetHeading,
            boolean isAutoTurning) {
        
        telemetry.addLine("========== TeleOp v2.1 ==========");
        telemetry.addData("运行时间", runtimeString);
        telemetry.addData("状态", "运行中");
        
        telemetry.addLine("");
        telemetry.addLine("--- 发射系统 ---");
        telemetry.addData("目标转速", "%d RPM", targetRPM);
        telemetry.addData("当前转速", "%.0f RPM", currentRPM);
        telemetry.addData("转速达标", isAtTargetSpeed ? "✓ 是" : "✗ 否");
        
        String fireStateStr;
        switch (fireState) {
            case RobotConstants_2_1.STATE_IDLE:
                fireStateStr = "待机";
                break;
            case RobotConstants_2_1.STATE_PREPARING:
                fireStateStr = "准备中";
                break;
            case RobotConstants_2_1.STATE_READY_TO_FIRE:
                fireStateStr = "准备好";
                break;
            case RobotConstants_2_1.STATE_FIRING:
                fireStateStr = "发射中";
                break;
            default:
                fireStateStr = "未知";
        }
        telemetry.addData("发射状态", fireStateStr);
        
        telemetry.addLine("");
        telemetry.addLine("--- 导航系统 ---");
        telemetry.addData("当前航向", "%.1f°", currentHeading);
        telemetry.addData("目标航向", "%.1f°", targetHeading);
        telemetry.addData("自动转向", isAutoTurning ? "进行中" : "关闭");
    }
    
    /**
     * 更新遥测显示
     */
    public void update() {
        telemetry.update();
    }
    
    /**
     * 清空遥测显示
     */
    public void clear() {
        telemetry.clear();
    }
}
