package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * TeleOp v2.0.0 - 主程序（多文件模块化版本）
 * 
 * 基于 v1.0 扩展，新增功能：
 * - IMU 自动转向控制（PID）
 * - 双发射电机独立驱动
 * - 多转速档位预设（4个）
 * - 转速精度自适应
 * - 状态机控制发射流程
 * 
 * 模块组织：
 * - RobotConstants_2_0: 常数管理
 * - ChassisDriveSystem_2_0: 底盘驱动
 * - NavigationSystem_2_0: IMU 导航
 * - SubsystemManager_2_0: 子系统控制
 * - ControlInputManager_2_0: 输入处理
 * - TelemetryManager_2_0: 遥测显示
 */
@TeleOp(name = "TeleOp_2_0", group = "TeleOp")
public class TeleOp_2_0 extends LinearOpMode {
    
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
    private ChassisDriveSystem_2_0 chassis;
    private NavigationSystem_2_0 navigation;
    private SubsystemManager_2_0 subsystems;
    private ControlInputManager_2_0 controlInput;
    private TelemetryManager_2_0 telemetryMgr;
    
    // ========== 运行时数据 ==========
    private ElapsedTime runtime;
    private boolean lastYState = false;
    
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
            motorsChassisFL = hardwareMap.get(DcMotor.class, RobotConstants_2_0.CHASSIS_MOTOR_FRONT_LEFT_NAME);
            motorsChassisFR = hardwareMap.get(DcMotor.class, RobotConstants_2_0.CHASSIS_MOTOR_FRONT_RIGHT_NAME);
            motorsChassiBL = hardwareMap.get(DcMotor.class, RobotConstants_2_0.CHASSIS_MOTOR_BACK_LEFT_NAME);
            motorsChassiBR = hardwareMap.get(DcMotor.class, RobotConstants_2_0.CHASSIS_MOTOR_BACK_RIGHT_NAME);
            
            motorIntake = hardwareMap.get(DcMotor.class, RobotConstants_2_0.SUBSYSTEM_INTAKE_MOTOR_NAME);
            motorLoad = hardwareMap.get(DcMotor.class, RobotConstants_2_0.SUBSYSTEM_LOAD_MOTOR_NAME);
            motorShooter1 = hardwareMap.get(DcMotorEx.class, RobotConstants_2_0.SUBSYSTEM_SHOOTER1_MOTOR_NAME);
            motorShooter2 = hardwareMap.get(DcMotorEx.class, RobotConstants_2_0.SUBSYSTEM_SHOOTER2_MOTOR_NAME);
            
            imu = hardwareMap.get(IMU.class, RobotConstants_2_0.IMU_SENSOR_NAME);
        } catch (IllegalArgumentException e) {
            telemetry.addData("初始化错误", "硬件配置不匹配: " + e.getMessage());
            telemetry.update();
            return;
        }
        
        // 初始化模块
        chassis = new ChassisDriveSystem_2_0(motorsChassisFL, motorsChassisFR, motorsChassiBL, motorsChassiBR);
        navigation = new NavigationSystem_2_0(imu, RobotConstants_2_0.IMU_SENSOR_NAME);
        subsystems = new SubsystemManager_2_0(motorIntake, motorLoad, motorShooter1, motorShooter2);
        controlInput = new ControlInputManager_2_0(gamepad1);
        telemetryMgr = new TelemetryManager_2_0(telemetry);
        
        // 初始化各系统
        chassis.initialize();
        navigation.initialize();
        subsystems.initialize();
        
        telemetry.addData("状态", "初始化完成");
        telemetry.update();
    }
    
    /**
     * 更新控制输入处理
     */
    private void updateControlInputs() {
        // 检查自动转向请求
        if (controlInput.isRightBumperPressed()) {
            navigation.startAutoTurn(RobotConstants_2_0.AUTO_TURN_TARGET_RIGHT);
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
        // 拾取和装填控制
        if (controlInput.isIntakeForwardRequested()) {
            subsystems.intakeStart();
            subsystems.loadStop();
        } else if (controlInput.isIntakeReverseRequested()) {
            subsystems.intakeReverse();
            subsystems.loadReverse();
        } else if (controlInput.isStopRequested()) {
            subsystems.intakeStop();
            subsystems.loadStop();
            subsystems.stopShooter();
        }
        
        // 发射转速档位选择
        if (controlInput.isShooterSpeedLongRangeRequested()) {
            subsystems.setShooterTargetRPM(RobotConstants_2_0.SHOOTER_RPM_LONG_RANGE);
        } else if (controlInput.isShooterSpeedSideRequested()) {
            subsystems.setShooterTargetRPM(RobotConstants_2_0.SHOOTER_RPM_TRIANGLE_SIDE);
        } else if (controlInput.isShooterSpeedBaseRequested()) {
            subsystems.setShooterTargetRPM(RobotConstants_2_0.SHOOTER_RPM_TRIANGLE_BASE);
        } else if (controlInput.isShooterSpeedTopRequested()) {
            subsystems.setShooterTargetRPM(RobotConstants_2_0.SHOOTER_RPM_TRIANGLE_TOP);
        }
        
        // 发射流程控制（Y 键边缘检测）
        boolean currentYState = controlInput.getYButtonState();
        subsystems.updateFireState(currentYState, lastYState);
        lastYState = currentYState;
    }
    
    /**
     * 更新遥测显示
     */
    private void updateTelemetry() {
        telemetryMgr.clear();
        
        telemetryMgr.displayFullTelemetry(
            runtime.toString(),
            subsystems.getTargetRPM(),
            subsystems.getCurrentShooterRPM(),
            subsystems.isShooterAtTargetSpeed(),
            subsystems.getFireState(),
            navigation.getCurrentHeading(),
            navigation.getCurrentHeading(),  // 目标航向暂时与当前相同
            navigation.isAutoTurning()
        );
        
        telemetryMgr.update();
    }
}
