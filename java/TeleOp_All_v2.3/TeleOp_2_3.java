package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * TeleOp v2.3.0 - 主程序（多文件模块化版本）
 * 
 * 基于 v2.2 升级，新增功能：
 * - 无头模式 (Field-Centric) 切换 (X键)
 * - IMU 航向重置 (Back键)
 */
@TeleOp(name = "TeleOp_2_3", group = "TeleOp")
public class TeleOp_2_3 extends LinearOpMode {
    
    // ========== 硬件对象 ==========
    private DcMotor motorsChassisFL;
    private DcMotor motorsChassisFR;
    private DcMotor motorsChassiBL;
    private DcMotor motorsChassiBR;
    private DcMotor motorIntake;
    private DcMotor motorLoad;
    private DcMotorEx motorShooter1;
    private DcMotorEx motorShooter2;
    private IMU imu;
    
    // ========== 模块对象 ==========
    private ChassisDriveSystem_2_3 chassis;
    private NavigationSystem_2_3 navigation;
    private SubsystemManager_2_3 subsystems;
    private ControlInputManager_2_3 controlInput;
    private TelemetryManager_2_3 telemetryMgr;
    
    // ========== 运行时数据 ==========
    private ElapsedTime runtime;
    
    @Override
    public void runOpMode() {
        // 初始化计时器
        runtime = new ElapsedTime();
        
        // 初始化所有硬件
        initializeAllSystems();
        
        // 等待游戏开始
        waitForStart();
        
        // 重置运行时间
        runtime.reset();
        
        // 主循环
        while (opModeIsActive()) {
            // 第一步：更新输入
            updateControlInputs();
            
            // 第二步：更新底盘运动
            updateChassisMovement();
            
            // 第三步：更新子系统
            updateSubsystemStates();
            
            // 第四步：更新遥测显示
            updateTelemetry();
        }
        
        // 停止所有系统
        chassis.stop();
        subsystems.stopAll();
    }
    
    /**
     * 初始化所有系统和模块
     */
    private void initializeAllSystems() {
        // 初始化硬件
        try {
            motorsChassisFL = hardwareMap.get(DcMotor.class, RobotConstants_2_3.CHASSIS_MOTOR_FRONT_LEFT_NAME);
            motorsChassisFR = hardwareMap.get(DcMotor.class, RobotConstants_2_3.CHASSIS_MOTOR_FRONT_RIGHT_NAME);
            motorsChassiBL = hardwareMap.get(DcMotor.class, RobotConstants_2_3.CHASSIS_MOTOR_BACK_LEFT_NAME);
            motorsChassiBR = hardwareMap.get(DcMotor.class, RobotConstants_2_3.CHASSIS_MOTOR_BACK_RIGHT_NAME);
            
            motorIntake = hardwareMap.get(DcMotor.class, RobotConstants_2_3.SUBSYSTEM_INTAKE_MOTOR_NAME);
            motorLoad = hardwareMap.get(DcMotor.class, RobotConstants_2_3.SUBSYSTEM_LOAD_MOTOR_NAME);
            motorShooter1 = hardwareMap.get(DcMotorEx.class, RobotConstants_2_3.SUBSYSTEM_SHOOTER1_MOTOR_NAME);
            motorShooter2 = hardwareMap.get(DcMotorEx.class, RobotConstants_2_3.SUBSYSTEM_SHOOTER2_MOTOR_NAME);
            
            imu = hardwareMap.get(IMU.class, RobotConstants_2_3.IMU_SENSOR_NAME);
        } catch (IllegalArgumentException e) {
            telemetry.addData("初始化错误", "硬件配置不匹配: " + e.getMessage());
            telemetry.update();
            return;
        }
        
        // 初始化模块
        // 注意：ChassisDriveSystem_2_3 依赖 NavigationSystem_2_3
        navigation = new NavigationSystem_2_3(imu, RobotConstants_2_3.IMU_SENSOR_NAME);
        chassis = new ChassisDriveSystem_2_3(motorsChassisFL, motorsChassisFR, motorsChassiBL, motorsChassiBR, navigation);
        subsystems = new SubsystemManager_2_3(motorIntake, motorLoad, motorShooter1, motorShooter2);
        controlInput = new ControlInputManager_2_3(gamepad1);
        telemetryMgr = new TelemetryManager_2_3(telemetry);
        
        // 初始化各系统
        navigation.initialize(); // 先初始化IMU
        chassis.initialize();
        subsystems.initialize();
        
        telemetry.addData("状态", "初始化完成 (v2.3)");
        telemetry.update();
    }
    
    /**
     * 更新控制输入处理
     */
    private void updateControlInputs() {
        // 自动转向请求
        if (controlInput.isRightBumperPressed()) {
            navigation.startAutoTurn(RobotConstants_2_3.AUTO_TURN_TARGET_RIGHT);
        }
        
        // 底盘模式切换 (线性/非线性)
        if (controlInput.isYButtonPressed()) {
            chassis.toggleDriveMode();
            controlInput.rumble();
        }

        // 无头模式切换 (有头/无头)
        if (controlInput.isXButtonPressed()) {
            chassis.toggleCentricMode();
            controlInput.rumble();
        }

        // 重置 IMU 航向
        if (controlInput.isOptionsPressed()) {
            navigation.resetYaw();
            controlInput.rumble();
        }
    }
    
    /**
     * 更新底盘运动
     */
    private void updateChassisMovement() {
        double drive = controlInput.getChassisDriveFBInput();
        double strafe = controlInput.getChassisStrafeLRInput();
        double turn = controlInput.getChassisRotateCWInput();
        
        // 计算自动转向功率
        double autoTurnPower = navigation.calculateAutoTurnPower();
        
        // 更新底盘
        chassis.update(drive, strafe, turn, navigation.isAutoTurning(), autoTurnPower);
    }
    
    /**
     * 更新子系统状态
     */
    private void updateSubsystemStates() {
        // ===== 拾取系统 =====
        if (controlInput.isIntakeForwardRequested()) {
            subsystems.intakeStart();
        } else if (controlInput.isIntakeReverseRequested()) {
            subsystems.intakeReverse();
        } else {
            subsystems.intakeStop();
        }
        
        // ===== 装填系统 =====
        if (controlInput.isLoadReverseRequested()) {
            subsystems.loadReverse();
        } else if (controlInput.isLoadRequested()) {
            subsystems.loadStart();
        } else {
            subsystems.loadStop();
        }
        
        // ===== 发射系统 (转速设置) =====
        boolean rpmChanged = false;
        if (controlInput.isShooterSpeedLongRangeRequested()) {
            subsystems.setShooterTargetRPM(RobotConstants_2_3.SHOOTER_RPM_LONG_RANGE);
            rpmChanged = true;
        } else if (controlInput.isShooterSpeedSideRequested()) {
            subsystems.setShooterTargetRPM(RobotConstants_2_3.SHOOTER_RPM_TRIANGLE_SIDE);
            rpmChanged = true;
        } else if (controlInput.isShooterSpeedBaseRequested()) {
            subsystems.setShooterTargetRPM(RobotConstants_2_3.SHOOTER_RPM_TRIANGLE_BASE);
            rpmChanged = true;
        } else if (controlInput.isShooterSpeedTopRequested()) {
            subsystems.setShooterTargetRPM(RobotConstants_2_3.SHOOTER_RPM_TRIANGLE_TOP);
            rpmChanged = true;
        }
        
        if (rpmChanged) {
            controlInput.rumble();
        }
        
        // ===== 发射系统 (激活控制) =====
        subsystems.setShootingState(controlInput.isShooterRequested());
    }
    
    /**
     * 更新遥测显示
     */
    private void updateTelemetry() {
        telemetryMgr.clear();
        
        telemetryMgr.displayFullTelemetry(
            String.format("%.1f s", runtime.seconds()),
            subsystems.getTargetRPM(),
            subsystems.getShooter1RPM(),
            subsystems.getShooter2RPM(),
            subsystems.isShooterAtTargetSpeed(),
            chassis.getDriveModeName(),
            subsystems.getIntakeStatus(),
            subsystems.getLoadStatus(),
            navigation.getHeading(),
            navigation.isAutoTurning()
        );
        
        telemetryMgr.update();
    }
}
