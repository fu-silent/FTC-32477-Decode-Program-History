package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * 遥测和调试信息管理类
 * 负责将机器人状态信息显示在手柄屏幕上
 * 职责：
 * - 收集各个系统的状态信息
 * - 格式化并显示遥测数据
 * - 提供实时调试信息
 */
public class TelemetryManager_1_0 {

    // ==================== 硬件对象 ====================
    private LinearOpMode opModeContext;

    // ==================== 构造函数 ====================
    public TelemetryManager_1_0(LinearOpMode opMode) {
        this.opModeContext = opMode;
    }

    // ==================== 遥测显示方法 ====================

    /**
     * 显示初始化状态
     */
    public void displayInitializationStatus(String chassisStatus, String subsystemStatus) {
        opModeContext.telemetry.addData("=== 初始化状态 ===", "");
        opModeContext.telemetry.addData("底盘系统", chassisStatus);
        opModeContext.telemetry.addData("子系统", subsystemStatus);
        opModeContext.telemetry.update();
    }

    /**
     * 显示等待开始界面
     */
    public void displayWaitingForStart() {
        opModeContext.telemetry.addData("状态", "等待开始...");
        opModeContext.telemetry.addData("提示", "按START按钮开始比赛");
        opModeContext.telemetry.update();
    }

    /**
     * 显示运行时的完整状态信息
     */
        public void displayRuntimeStatus(
            ControlInputManager_1_0 controlInput,
            ChassisDriveSystem_1_0 chassis,
            SubsystemManager_1_0 subsystem) {

        // 获取当前数据
        double[] wheelPowers = chassis.getCurrentWheelPowers();
        double shooterPower = subsystem.getShooterPower();

        // 显示底盘信息
        opModeContext.telemetry.addData("=== 底盘控制 ===", "");
        opModeContext.telemetry.addData("映射模式", 
            controlInput.isChassisMappingNonlinearEnabled() ? "非线性(平方)" : "线性");
        opModeContext.telemetry.addData("前后/左右/旋转", 
            String.format("%.2f / %.2f / %.2f", 
                controlInput.getChassisDriveFBInput(),
                controlInput.getChassisStrafeLRInput(),
                controlInput.getChassisRotateCWInput()));
        opModeContext.telemetry.addData("电机功率(FL/FR/BL/BR)", 
            String.format("%.2f / %.2f / %.2f / %.2f", 
                wheelPowers[0], wheelPowers[1], wheelPowers[2], wheelPowers[3]));

        // 显示发射系统信息（功率控制）
        opModeContext.telemetry.addData("=== 发射模块 ===", "");
        String shooterState = controlInput.isShooterRequested() ? "发射中" : "关闭";
        opModeContext.telemetry.addData("状态", shooterState);
        opModeContext.telemetry.addData("发射功率", String.format("%.2f", shooterPower));

        // 显示装填系统信息
        opModeContext.telemetry.addData("=== 装填模块 ===", "");
        opModeContext.telemetry.addData("状态", 
            controlInput.isLoadRequested() ? "运行中" : "停止");
        opModeContext.telemetry.addData("功率", String.format("%.2f", subsystem.getLoadPower()));

        // 显示拾取系统信息
        opModeContext.telemetry.addData("=== 拾取模块 ===", "");
        String intakeState;
        if (controlInput.isIntakeForwardRequested()) {
            intakeState = "正向";
        } else if (controlInput.isIntakeReverseRequested()) {
            intakeState = "反向";
        } else {
            intakeState = "停止";
        }
        opModeContext.telemetry.addData("状态", intakeState);
        opModeContext.telemetry.addData("功率", String.format("%.2f", subsystem.getIntakePower()));

        // 显示按键说明
        opModeContext.telemetry.addData("=== 按键参考 ===", "");
        opModeContext.telemetry.addData("A/B", "拾取 正向/反向");
        opModeContext.telemetry.addData("LT/LB", "装填 正向/反向");
        opModeContext.telemetry.addData("RT", "发射模块");
        opModeContext.telemetry.addData("D-Pad ↑/↓", "发射功率 +/-0.05");
        opModeContext.telemetry.addData("D-Pad ←/→", "发射功率 +/-0.02");
        opModeContext.telemetry.addData("Y", "切换映射模式");

        opModeContext.telemetry.update();
    }

    /**
     * 显示错误信息
     */
    public void displayError(String errorMessage) {
        opModeContext.telemetry.addData("错误", errorMessage);
        opModeContext.telemetry.update();
    }

    /**
     * 添加自定义调试信息
     */
    public void addDebugData(String key, Object value) {
        opModeContext.telemetry.addData(key, value);
    }

    /**
     * 更新遥测显示
     */
    public void update() {
        opModeContext.telemetry.update();
    }
}
