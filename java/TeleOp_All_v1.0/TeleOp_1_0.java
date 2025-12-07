package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * FTC32477 机器人 TeleOp 远程控制程序（模块化版 v1.0.0）
 * 
 * 赛季主题：Decode
 * 
 * 程序架构：
 * 本程序采用模块化设计，将复杂的控制逻辑分解为独立的模块，各司其职：
 * 
 * 1. RobotConstants - 常量管理
 *    集中管理所有硬件配置、参数常数
 *    便于维护和调整参数
 * 
 * 2. ChassisDriveSystem - 底盘驱动系统
 *    麦克纳姆轮电机管理、运动学计算、死区/非线性映射
 * 
 * 3. SubsystemManager - 子系统管理
 *    拾取(Intake)、装填(Load)、发射(Shooter)三个独立系统
 * 
 * 4. ControlInputManager - 控制输入处理
 *    手柄摇杆/按键/扳机输入、边沿检测、模式切换
 * 
 * 5. TelemetryManager - 遥测显示
 *    收集和显示所有系统的状态信息
 * 
 * 控制方案：
 * -------
 * 底盘控制：
 *   - 左摇杆 Y 轴：前后移动
 *   - 左摇杆 X 轴：左右平移
 *   - 右摇杆 X 轴：原地旋转（灵敏度 80%）
 *   - Y 键：切换线性/非线性映射模式
 * 
 * 拾取系统(Intake)：
 *   - A 键：正向转动
 *   - B 键：反向转动
 * 
 * 装填系统(Load)：
 *   - LT（左扳机）：启动装填（超过0.1死区）
 *   - LB（左肩键）：装填反转
 * 
 * 发射系统(Shooter)：
 *   - RT（右扳机）：启动发射（超过0.1死区）
 *   - D-Pad 上：目标转速 +50 RPM
 *   - D-Pad 下：目标转速 -50 RPM
 * 
 * 硬件配置：
 * --------
 * 底盘：4 个麦克纳姆轮电机（前左、前右、后左、后右）
 * 拾取：1 个 DcMotor
 * 装填：1 个 DcMotor
 * 发射：2 个 DcMotorEx（用于闭环控制）
 * 
 * @author FTC32477
 * @version 1.0.0 - 完整模块化重构版
 */
@TeleOp(name = "TeleOp_1_0 - Modular", group = "TeleOp")
public class TeleOp_1_0 extends LinearOpMode {

    // ==================== 模块实例 ====================
    private ChassisDriveSystem_1_0 chassisSystem;
    private SubsystemManager_1_0 subsystemManager;
    private ControlInputManager_1_0 controlInput;
    private TelemetryManager_1_0 telemetryDisplay;

    // ==================== 程序主入口 ====================
    @Override
    public void runOpMode() {
        // 初始化所有系统模块
        if (!initializeAllSystems()) {
            telemetryDisplay.displayError("系统初始化失败，请检查硬件连接");
            return;
        }

        // 显示等待开始
        telemetryDisplay.displayWaitingForStart();

        // 等待比赛开始信号
        waitForStart();

        // 主控制循环
        while (opModeIsActive()) {
            // 步骤1：读取并处理控制输入
            updateControlInputs();

            // 步骤2：计算底盘运动
            updateChassisMovement();

            // 步骤3：更新子系统状态
            updateSubsystemStates();

            // 步骤4：更新遥测显示
            updateTelemetry();
        }

        // 停止所有系统
        chassisSystem.stop();
        subsystemManager.stopAll();
    }

    // ==================== 系统初始化 ====================

    /**
     * 初始化所有系统模块
     * @return 初始化是否成功
     */
    private boolean initializeAllSystems() {
        try {
            telemetry.addData("状态", "正在初始化系统...");
            telemetry.update();

            // 初始化模块实例
            chassisSystem = new ChassisDriveSystem_1_0();
            subsystemManager = new SubsystemManager_1_0();
            controlInput = new ControlInputManager_1_0(gamepad1);
            telemetryDisplay = new TelemetryManager_1_0(this);

            // 初始化底盘系统
            if (!chassisSystem.initialize(hardwareMap)) {
                telemetry.addData("错误", "底盘系统初始化失败");
                telemetry.addData("检查项", "检查硬件映射名称: lf, rf, lb, rb");
                telemetry.update();
                return false;
            }

            // 初始化子系统
            if (!subsystemManager.initialize(hardwareMap)) {
                telemetry.addData("错误", "子系统初始化失败");
                telemetry.addData("检查项", "检查硬件映射名称: intake, load, s1, s2");
                telemetry.update();
                return false;
            }

            // 显示初始化成功信息
            telemetryDisplay.displayInitializationStatus(
                chassisSystem.getInitializationStatus(),
                subsystemManager.getInitializationStatus()
            );

            return true;

        } catch (Exception e) {
            telemetry.addData("异常", e.getMessage());
            telemetry.update();
            return false;
        }
    }

    // ==================== 控制循环步骤 ====================

    /**
     * 步骤1：读取和处理控制输入
     * 更新所有按键状态、检查模式切换
     */
    private void updateControlInputs() {
        // 检查映射模式切换请求
        if (controlInput.isMappingModeToggleRequested()) {
            controlInput.toggleChassisMappingMode();
            subsystemManager.setShooterPower(RobotConstants_1_0.SHOOTER_POWER_DEFAULT);
        }

        // 检查发射速度调节请求
        if (controlInput.isShooterPowerIncreaseRequested()) {
            subsystemManager.increaseShooterPower();
        }
        if (controlInput.isShooterPowerDecreaseRequested()) {
            subsystemManager.decreaseShooterPower();
        }
        if (controlInput.isShooterPowerRightRequested()) {
            subsystemManager.increaseShooterPowerSmall();
        }
        if (controlInput.isShooterPowerLeftRequested()) {
            subsystemManager.decreaseShooterPowerSmall();
        }

        // 更新按键状态（便于将来扩展）
        controlInput.updateButtonStates();
    }

    /**
     * 步骤2：更新底盘运动
     * 读取摇杆输入，计算四轮功率，控制电机
     */
    private void updateChassisMovement() {
        // 获取控制输入
        double inputDriveFB = controlInput.getChassisDriveFBInput();
        double inputStrafeLR = controlInput.getChassisStrafeLRInput();
        double inputRotateCW = controlInput.getChassisRotateCWInput();

        // 计算四轮功率（应用非线性映射）
        double[] wheelPowers = chassisSystem.calculateWheelPowers(
            inputDriveFB,
            inputStrafeLR,
            inputRotateCW,
            controlInput.isChassisMappingNonlinearEnabled()
        );

        // 设置电机功率
        chassisSystem.setWheelPowers(wheelPowers);
    }

    /**
     * 步骤3：更新子系统状态
     * 根据按键/扳机输入控制拾取、装填、发射三个系统
     */
    private void updateSubsystemStates() {
        // ===== 拾取系统(Intake)控制 =====
        if (controlInput.isIntakeForwardRequested()) {
            subsystemManager.intakeStart();
        } else if (controlInput.isIntakeReverseRequested()) {
            subsystemManager.intakeReverse();
        } else {
            subsystemManager.intakeStop();
        }

        // ===== 装填系统(Load)控制 =====
        if (controlInput.isLoadReverseRequested()) {
            subsystemManager.loadReverse();
        } else if (controlInput.isLoadRequested()) {
            subsystemManager.loadStart();
        } else {
            subsystemManager.loadStop();
        }

        // ===== 发射系统(Shooter)控制 =====
        if (controlInput.isShooterRequested()) {
            subsystemManager.shooterStart();
        } else {
            subsystemManager.shooterStop();
        }
    }

    /**
     * 步骤4：更新遥测显示
     * 将所有系统状态显示在手柄屏幕上
     */
    private void updateTelemetry() {
        telemetryDisplay.displayRuntimeStatus(controlInput, chassisSystem, subsystemManager);
    }
}
