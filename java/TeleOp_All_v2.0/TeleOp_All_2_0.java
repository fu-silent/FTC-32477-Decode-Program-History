package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * TeleOp v2.0.0 - 单文件集成版本
 * 
 * 所有功能集成在一个文件中，使用内部类模式组织代码
 * 
 * 功能：
 * - IMU 自动转向控制（PID）
 * - 双发射电机独立驱动
 * - 多转速档位预设（4个）
 * - 转速精度自适应
 * - 状态机控制发射流程
 */
@TeleOp(name = "TeleOp_All_2_0", group = "TeleOp")
public class TeleOp_All_2_0 extends LinearOpMode {
    
    // ========== 内部类：常数管理 ==========
    class Constants {
        // 硬件名称
        final String CHASSIS_MOTOR_FRONT_LEFT_NAME = "lf";
        final String CHASSIS_MOTOR_FRONT_RIGHT_NAME = "rf";
        final String CHASSIS_MOTOR_BACK_LEFT_NAME = "lb";
        final String CHASSIS_MOTOR_BACK_RIGHT_NAME = "rb";
        final String SUBSYSTEM_INTAKE_MOTOR_NAME = "intake";
        final String SUBSYSTEM_LOAD_MOTOR_NAME = "load";
        final String SUBSYSTEM_SHOOTER1_MOTOR_NAME = "s1";
        final String SUBSYSTEM_SHOOTER2_MOTOR_NAME = "s2";
        final String IMU_SENSOR_NAME = "imu";
        
        // 底盘参数
        final double CHASSIS_JOYSTICK_DEADZONE = 0.1;
        final double TURN_SENSITIVITY_FACTOR = 0.8;
        
        // 发射参数
        final double SHOOTER_MOTOR_TICK_COUNT = 28;
        final double SHOOTER_PIDF_P = 135;
        final double SHOOTER_PIDF_I = 0;
        final double SHOOTER_PIDF_D = 80;
        final double SHOOTER_PIDF_F = 14;
        
        // 转速档位
        final int SHOOTER_RPM_LONG_RANGE = 3200;
        final int SHOOTER_RPM_TRIANGLE_SIDE = 1900;
        final int SHOOTER_RPM_TRIANGLE_BASE = 1650;
        final int SHOOTER_RPM_TRIANGLE_TOP = 2400;
        
        // 精度范围
        final int SHOOTER_RPM_ERROR_RANGE_LONG = 30;
        final int SHOOTER_RPM_ERROR_RANGE_SIDE = 50;
        final int SHOOTER_RPM_ERROR_RANGE_BASE = 150;
        final int SHOOTER_RPM_ERROR_RANGE_TOP = 35;
        
        // 电机功率
        final double INTAKE_FORWARD_POWER = 0.8;
        final double INTAKE_REVERSE_POWER = -0.8;
        final double LOAD_FORWARD_POWER = 0.55;
        final double LOAD_REVERSE_POWER = -0.55;
        
        // IMU 参数
        final double AUTO_TURN_POWER = 0.5;
        final double AUTO_TURN_HEADING_THRESHOLD = 2.0;
        final double AUTO_TURN_P_GAIN = 0.1;
        final double AUTO_TURN_I_GAIN = 0.0;
        final double AUTO_TURN_D_GAIN = 0.005;
        final double AUTO_TURN_TARGET_RIGHT = 45.0;
        
        // 状态
        final int STATE_IDLE = 0;
        final int STATE_PREPARING = 1;
        final int STATE_READY_TO_FIRE = 2;
        final int STATE_FIRING = 3;
    }
    
    // ========== 内部类：底盘驱动 ==========
    class ChassisDrive {
        DcMotor motorFL, motorFR, motorBL, motorBR;
        
        ChassisDrive(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {
            this.motorFL = fl;
            this.motorFR = fr;
            this.motorBL = bl;
            this.motorBR = br;
        }
        
        void initialize() {
            motorFL.setDirection(DcMotor.Direction.FORWARD);
            motorFR.setDirection(DcMotor.Direction.REVERSE);
            motorBL.setDirection(DcMotor.Direction.REVERSE);
            motorBR.setDirection(DcMotor.Direction.FORWARD);
            motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        
        void update(double drive, double strafe, double turn, 
                    boolean autoTurning, double autoTurnPower) {
            drive = applyDeadzone(drive);
            strafe = applyDeadzone(strafe);
            
            if (autoTurning) {
                turn = autoTurnPower;
            } else {
                turn = applyDeadzone(turn);
                turn = Math.copySign(turn * turn, turn);
            }
            
            double flPower = drive + strafe + turn;
            double frPower = drive - strafe - turn;
            double blPower = drive - strafe + turn;
            double brPower = drive + strafe - turn;
            
            normalizeAndApply(flPower, frPower, blPower, brPower);
        }
        
        double applyDeadzone(double input) {
            return Math.abs(input) < constants.CHASSIS_JOYSTICK_DEADZONE ? 0 : input;
        }
        
        void normalizeAndApply(double fl, double fr, double bl, double br) {
            double maxPower = Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))
            );
            if (maxPower > 1.0) {
                fl /= maxPower;
                fr /= maxPower;
                bl /= maxPower;
                br /= maxPower;
            }
            motorFL.setPower(fl);
            motorFR.setPower(fr);
            motorBL.setPower(bl);
            motorBR.setPower(br);
        }
        
        void stop() {
            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBL.setPower(0);
            motorBR.setPower(0);
        }
    }
    
    // ========== 内部类：子系统管理 ==========
    class Subsystems {
        DcMotor motorIntake, motorLoad;
        DcMotorEx motorShooter1, motorShooter2;
        int targetRPM = 0;
        int errorRange = 50;
        int fireState = 0;
        
        Subsystems(DcMotor intake, DcMotor load, DcMotorEx s1, DcMotorEx s2) {
            this.motorIntake = intake;
            this.motorLoad = load;
            this.motorShooter1 = s1;
            this.motorShooter2 = s2;
        }
        
        void initialize() {
            motorIntake.setDirection(DcMotor.Direction.FORWARD);
            motorLoad.setDirection(DcMotor.Direction.REVERSE);
            motorShooter1.setDirection(DcMotor.Direction.FORWARD);
            motorShooter2.setDirection(DcMotor.Direction.REVERSE);
            motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorLoad.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorShooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorShooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
            motorShooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorShooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            PIDFCoefficients pidf = new PIDFCoefficients(
                constants.SHOOTER_PIDF_P,
                constants.SHOOTER_PIDF_I,
                constants.SHOOTER_PIDF_D,
                constants.SHOOTER_PIDF_F
            );
            motorShooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            motorShooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        }
        
        void setShooterRPM(int rpm) {
            this.targetRPM = rpm;
            if (rpm == constants.SHOOTER_RPM_LONG_RANGE) {
                this.errorRange = constants.SHOOTER_RPM_ERROR_RANGE_LONG;
            } else if (rpm == constants.SHOOTER_RPM_TRIANGLE_SIDE) {
                this.errorRange = constants.SHOOTER_RPM_ERROR_RANGE_SIDE;
            } else if (rpm == constants.SHOOTER_RPM_TRIANGLE_BASE) {
                this.errorRange = constants.SHOOTER_RPM_ERROR_RANGE_BASE;
            } else if (rpm == constants.SHOOTER_RPM_TRIANGLE_TOP) {
                this.errorRange = constants.SHOOTER_RPM_ERROR_RANGE_TOP;
            }
            updateShooterSpeed();
        }
        
        void updateShooterSpeed() {
            double ticks = targetRPM * constants.SHOOTER_MOTOR_TICK_COUNT / 60.0;
            motorShooter1.setVelocity(ticks);
            motorShooter2.setVelocity(ticks);
        }
        
        double getCurrentRPM() {
            return (motorShooter2.getVelocity() / constants.SHOOTER_MOTOR_TICK_COUNT) * 60.0;
        }
        
        boolean isAtTargetSpeed() {
            double rpm = getCurrentRPM();
            return rpm >= (targetRPM - errorRange) && rpm <= (targetRPM + errorRange);
        }
        
        void intakeStart() { motorIntake.setPower(constants.INTAKE_FORWARD_POWER); }
        void intakeReverse() { motorIntake.setPower(constants.INTAKE_REVERSE_POWER); }
        void intakeStop() { motorIntake.setPower(0); }
        void loadStart() { motorLoad.setPower(constants.LOAD_FORWARD_POWER); }
        void loadReverse() { motorLoad.setPower(constants.LOAD_REVERSE_POWER); }
        void loadStop() { motorLoad.setPower(0); }
        void stopShooter() { targetRPM = 0; updateShooterSpeed(); }
        void stopAll() { intakeStop(); loadStop(); stopShooter(); }
    }
    
    // ========== 内部类：导航系统 ==========
    class Navigation {
        IMU imu;
        boolean autoTurning = false;
        double targetHeading = 0;
        double integralSum = 0, prevError = 0;
        long prevTime = 0;
        
        Navigation(IMU imu) {
            this.imu = imu;
        }
        
        void initialize() {
            RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
            );
            imu.initialize(new IMU.Parameters(orientation));
            imu.resetYaw();
        }
        
        double getHeading() {
            return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }
        
        void startAutoTurn(double target) {
            targetHeading = target;
            autoTurning = true;
            resetPID();
        }
        
        double calculateTurnPower() {
            if (!autoTurning) return 0;
            
            double current = getHeading();
            double error = normalizeAngle(current - targetHeading);
            
            if (Math.abs(error) <= constants.AUTO_TURN_HEADING_THRESHOLD) {
                autoTurning = false;
                return 0;
            }
            
            long now = System.nanoTime();
            double dt = (now - prevTime) / 1e9;
            if (dt == 0) dt = 0.01;
            
            double prop = constants.AUTO_TURN_P_GAIN * error;
            integralSum += error * dt;
            double integral = constants.AUTO_TURN_I_GAIN * integralSum;
            double deriv = constants.AUTO_TURN_D_GAIN * (error - prevError) / dt;
            double output = Range.clip(prop + integral + deriv, -constants.AUTO_TURN_POWER, constants.AUTO_TURN_POWER);
            
            prevError = error;
            prevTime = now;
            return output;
        }
        
        double normalizeAngle(double a) {
            while (a > 180) a -= 360;
            while (a < -180) a += 360;
            return a;
        }
        
        void resetPID() {
            prevError = 0;
            integralSum = 0;
            prevTime = System.nanoTime();
        }
    }
    
    // ========== 内部类：输入处理 ==========
    class ControlInput {
        boolean lastY = false, lastBumper = false;
        
        double getDriveFB() { return -gamepad1.left_stick_y; }
        double getDriveLR() { return gamepad1.left_stick_x; }
        double getTurn() { return gamepad1.right_stick_x; }
        
        boolean getA() { return gamepad1.a; }
        boolean getB() { return gamepad1.b; }
        boolean getX() { return gamepad1.x; }
        
        boolean getDpadRight() { return gamepad1.dpad_right; }
        boolean getDpadLeft() { return gamepad1.dpad_left; }
        boolean getDpadDown() { return gamepad1.dpad_down; }
        boolean getDpadUp() { return gamepad1.dpad_up; }
        
        boolean getYPressed() {
            boolean current = gamepad1.y;
            boolean result = current && !lastY;
            lastY = current;
            return result;
        }
        
        boolean getRightBumperPressed() {
            boolean current = gamepad1.right_bumper;
            boolean result = current && !lastBumper;
            lastBumper = current;
            return result;
        }
    }
    
    // ========== 内部类：遥测显示 ==========
    class TelemetryDisplay {
        void displayFull(String runtime, int targetRPM, double currentRPM, 
                        boolean atSpeed, int fireState, double heading, boolean autoTurning) {
            telemetry.addLine("========== TeleOp v2.0 ==========");
            telemetry.addData("运行时间", runtime);
            telemetry.addLine("--- 发射系统 ---");
            telemetry.addData("目标转速", "%d RPM", targetRPM);
            telemetry.addData("当前转速", "%.0f RPM", currentRPM);
            telemetry.addData("转速达标", atSpeed ? "✓ 是" : "✗ 否");
            telemetry.addLine("--- 导航系统 ---");
            telemetry.addData("当前航向", "%.1f°", heading);
            telemetry.addData("自动转向", autoTurning ? "进行中" : "关闭");
        }
    }
    
    // ========== 成员变量 ==========
    Constants constants;
    ChassisDrive chassis;
    Subsystems subsystems;
    Navigation navigation;
    ControlInput controlInput;
    TelemetryDisplay telemetryDisplay;
    ElapsedTime runtime;
    
    @Override
    public void runOpMode() {
        constants = new Constants();
        runtime = new ElapsedTime();
        
        // 初始化硬件
        try {
            DcMotor fl = hardwareMap.get(DcMotor.class, constants.CHASSIS_MOTOR_FRONT_LEFT_NAME);
            DcMotor fr = hardwareMap.get(DcMotor.class, constants.CHASSIS_MOTOR_FRONT_RIGHT_NAME);
            DcMotor bl = hardwareMap.get(DcMotor.class, constants.CHASSIS_MOTOR_BACK_LEFT_NAME);
            DcMotor br = hardwareMap.get(DcMotor.class, constants.CHASSIS_MOTOR_BACK_RIGHT_NAME);
            DcMotor intake = hardwareMap.get(DcMotor.class, constants.SUBSYSTEM_INTAKE_MOTOR_NAME);
            DcMotor load = hardwareMap.get(DcMotor.class, constants.SUBSYSTEM_LOAD_MOTOR_NAME);
            DcMotorEx s1 = hardwareMap.get(DcMotorEx.class, constants.SUBSYSTEM_SHOOTER1_MOTOR_NAME);
            DcMotorEx s2 = hardwareMap.get(DcMotorEx.class, constants.SUBSYSTEM_SHOOTER2_MOTOR_NAME);
            IMU imu = hardwareMap.get(IMU.class, constants.IMU_SENSOR_NAME);
            
            chassis = new ChassisDrive(fl, fr, bl, br);
            subsystems = new Subsystems(intake, load, s1, s2);
            navigation = new Navigation(imu);
            controlInput = new ControlInput();
            telemetryDisplay = new TelemetryDisplay();
            
            chassis.initialize();
            subsystems.initialize();
            navigation.initialize();
        } catch (Exception e) {
            telemetry.addData("初始化错误", e.getMessage());
            telemetry.update();
            return;
        }
        
        telemetry.addData("状态", "准备就绪");
        telemetry.update();
        
        waitForStart();
        runtime.reset();
        
        while (opModeIsActive()) {
            // 输入处理
            if (controlInput.getRightBumperPressed()) {
                navigation.startAutoTurn(constants.AUTO_TURN_TARGET_RIGHT);
            }
            
            // 底盘更新
            double autoTurnPower = navigation.calculateTurnPower();
            chassis.update(
                controlInput.getDriveFB(),
                controlInput.getDriveLR(),
                controlInput.getTurn(),
                navigation.autoTurning,
                autoTurnPower
            );
            
            // 子系统更新
            if (controlInput.getA()) {
                subsystems.intakeStart();
                subsystems.loadStop();
            } else if (controlInput.getB()) {
                subsystems.intakeReverse();
                subsystems.loadReverse();
            } else if (controlInput.getX()) {
                subsystems.intakeStop();
                subsystems.loadStop();
                subsystems.stopShooter();
            }
            
            if (controlInput.getDpadRight()) subsystems.setShooterRPM(constants.SHOOTER_RPM_LONG_RANGE);
            if (controlInput.getDpadLeft()) subsystems.setShooterRPM(constants.SHOOTER_RPM_TRIANGLE_SIDE);
            if (controlInput.getDpadDown()) subsystems.setShooterRPM(constants.SHOOTER_RPM_TRIANGLE_BASE);
            if (controlInput.getDpadUp()) subsystems.setShooterRPM(constants.SHOOTER_RPM_TRIANGLE_TOP);
            
            // 遥测
            telemetryDisplay.displayFull(
                runtime.toString(),
                subsystems.targetRPM,
                subsystems.getCurrentRPM(),
                subsystems.isAtTargetSpeed(),
                subsystems.fireState,
                navigation.getHeading(),
                navigation.autoTurning
            );
            telemetry.update();
        }
        
        chassis.stop();
        subsystems.stopAll();
    }
}
