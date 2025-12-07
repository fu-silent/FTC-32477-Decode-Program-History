package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * 遥测管理器 v2.2.0 - 系统状态显示
 * 
 * 显示内容：
 * - 底盘运动状态
 * - 发射系统状态和转速
 * - IMU 航向角
 * - 自动转向状态
 * - 按键提示
 */
public class TelemetryManager_2_2 {
    
    private final Telemetry telemetry;
    
    public TelemetryManager_2_2(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    
    /**
     * 显示运行时间和基本状态
     */
    public void displayRuntimeStatus(String runtimeString) {
        telemetry.addLine("========== TeleOp v2.2 ==========");
        telemetry.addData("运行时间", runtimeString);
        telemetry.addData("状态", "运行中");
    }
    
    /**
     * 显示完整的遥测信息（综合显示）
     */
    public void displayFullTelemetry(
            String runtimeString,
            int targetRPM,
            double s1RPM,
            double s2RPM,
            boolean atSpeed,
            String chassisMode,
            String intakeStatus,
            String loadStatus,
            double heading,
            boolean autoTurning) {
        
        telemetry.addLine("========== TeleOp v2.2 ==========");
        telemetry.addData("运行时间", runtimeString);
        
        telemetry.addLine("\n--- 发射系统 ---");
        telemetry.addData("目标转速", "%d RPM", targetRPM);
        telemetry.addData("S1转速", "%.0f RPM", s1RPM);
        telemetry.addData("S2转速", "%.0f RPM", s2RPM);
        telemetry.addData("转速达标", atSpeed ? "✓ 是" : "✗ 否");
        
        telemetry.addLine("\n--- 底盘与导航 ---");
        telemetry.addData("移动模式", chassisMode);
        telemetry.addData("当前航向", "%.1f°", heading);
        telemetry.addData("自动转向", autoTurning ? "进行中" : "关闭");
        
        telemetry.addLine("\n--- 状态 ---");
        telemetry.addData("拾取模块", intakeStatus);
        telemetry.addData("装填模块", loadStatus);
        
        telemetry.addLine("\n--- 按键说明 ---");
        telemetry.addLine("Y:切换底盘模式 | Dpad:预设转速");
        telemetry.addLine("RT:发射(需按住) | RB:自动转向");
        telemetry.addLine("A/B:拾取 | LT/LB:装填");
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
