package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * 遥测管理器 v2.3.0 - 系统状态显示
 * 
 * 新增功能：
 * - 显示无头模式状态
 * - 更新按键说明
 */
public class TelemetryManager_2_3 {
    
    private final Telemetry telemetry;
    
    public TelemetryManager_2_3(Telemetry telemetry) {
        this.telemetry = telemetry;
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
        
        telemetry.addLine("========== TeleOp v2.3 ==========");
        telemetry.addData("运行时间", runtimeString);
        
        telemetry.addLine("\n--- 发射系统 ---");
        telemetry.addData("目标转速", "%d RPM", targetRPM);
        telemetry.addData("S1转速", "%.0f RPM", s1RPM);
        telemetry.addData("S2转速", "%.0f RPM", s2RPM);
        telemetry.addData("转速达标", atSpeed ? "✓ 是" : "✗ 否");
        
        telemetry.addLine("\n--- 底盘与导航 ---");
        telemetry.addData("驱动模式", chassisMode);
        telemetry.addData("当前航向", "%.1f°", heading);
        telemetry.addData("自动转向", autoTurning ? "进行中" : "关闭");
        
        telemetry.addLine("\n--- 状态 ---");
        telemetry.addData("拾取模块", intakeStatus);
        telemetry.addData("装填模块", loadStatus);
        
        telemetry.addLine("\n--- 按键说明 ---");
        telemetry.addLine("Y:线性切换 | X:无头切换 | Back:重置IMU");
        telemetry.addLine("RT:发射 | RB:自动转向 | Dpad:档位");
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
