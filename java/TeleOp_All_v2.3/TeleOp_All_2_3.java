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

/**
 * TeleOp v2.3.0 - 无头模式 (Field-Centric) 集成版
 * * 新增功能：
 * - 支持有头/无头模式切换 (按 X 键)
 * - 支持 IMU 航向重置 (按 Back 键)
 * * 保留功能：
 * - 线性/非线性驱动切换 (按 Y 键)
 * - 所有发射、装填、自动转向原有逻辑
 */
@TeleOp(name = "TeleOp_All_2_3", group = "TeleOp")
public class TeleOp_All_2_3 extends LinearOpMode {

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

        // 发射参数
        final double SHOOTER_MOTOR_TICK_COUNT = 28;
        final double SHOOTER_PIDF_F = 14;
        final double SHOOTER_PIDF_P = 250;
        final double SHOOTER_PIDF_I = 0;
        final double SHOOTER_PIDF_D = 100;

        // 转速档位
        final int SHOOTER_RPM_LONG_RANGE = 1950;
        final int SHOOTER_RPM_TRIANGLE_SIDE = 1400;
        final int SHOOTER_RPM_TRIANGLE_BASE = 1200;
        final int SHOOTER_RPM_TRIANGLE_TOP = 1625;

        // 精度范围
        final int SHOOTER_RPM_ERROR_RANGE_LONG = 50;
        final int SHOOTER_RPM_ERROR_RANGE_SIDE = 50;
        final int SHOOTER_RPM_ERROR_RANGE_BASE = 50;
        final int SHOOTER_RPM_ERROR_RANGE_TOP = 50;

        // 电机功率
        final double INTAKE_FORWARD_POWER = 0.9;
        final double INTAKE_REVERSE_POWER = -0.9;
        final double LOAD_FORWARD_POWER = 0.75;
        final double LOAD_REVERSE_POWER = -0.75;

        // IMU 参数
        final double AUTO_TURN_POWER = 0.5;
        final double AUTO_TURN_HEADING_THRESHOLD = 2.0;
        final double AUTO_TURN_P_GAIN = 0.1;
        final double AUTO_TURN_I_GAIN = 0.0;
        final double AUTO_TURN_D_GAIN = 0.005;
        final double AUTO_TURN_TARGET_RIGHT = 45.0;

        // 手柄震动
        final int RUMBLE_DURATION_MS = 200;
    }

    // ========== 内部类：底盘驱动 (已修改支持无头模式) ==========
    class ChassisDrive {
        DcMotor motorFL, motorFR, motorBL, motorBR;
        Navigation navigation; // 引用导航系统以获取角度

        boolean isNonLinearMode = true; // 默认为非线性模式
        boolean isFieldCentric = true;  // 默认为无头模式 (True)

        // 修改构造函数，接收 Navigation 对象
        ChassisDrive(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, Navigation nav) {
            this.motorFL = fl;
            this.motorFR = fr;
            this.motorBL = bl;
            this.motorBR = br;
            this.navigation = nav;
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

        // 切换线性/非线性 (Y键)
        void toggleDriveMode() {
            isNonLinearMode = !isNonLinearMode;
        }

        // 切换有头/无头 (X键)
        void toggleCentricMode() {
            isFieldCentric = !isFieldCentric;
        }

        String getDriveModeName() {
            String centric = isFieldCentric ? "无头 (Field)" : "有头 (Robot)";
            String linear = isNonLinearMode ? "非线性" : "线性";
            return centric + " | " + linear;
        }

        void update(double drive, double strafe, double turn,
                    boolean autoTurning, double autoTurnPower) {

            // 1. 死区处理
            drive = applyDeadzone(drive);
            strafe = applyDeadzone(strafe);

            // 2. 转向输入处理
            if (autoTurning) {
                turn = autoTurnPower;
            } else {
                turn = applyDeadzone(turn);
                // 非线性处理
                if (isNonLinearMode) {
                    drive = Math.copySign(drive * drive, drive);
                    strafe = Math.copySign(strafe * strafe, strafe);
                    turn = Math.copySign(turn * turn, turn);
                }
            }

            // 3. 无头模式 (Field-Centric) 坐标转换
            // [依赖变量]: navigation (获取当前航向), drive/strafe (手柄输入)
            if (isFieldCentric) {
                double botHeading = Math.toRadians(navigation.getHeading());

                // 将手柄输入的 地面坐标系向量 旋转为 机器人坐标系向量
                // 旋转公式：x' = x*cos(-θ) - y*sin(-θ)
                double rotX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
                double rotY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);

                strafe = rotX;
                drive = rotY;
            }

            // 4. 麦克纳姆轮解算
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
        boolean isShooting = false;

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
            if (isShooting) {
                updateShooterSpeed();
            }
        }

        void setShootingState(boolean firing) {
            if (isShooting != firing) {
                isShooting = firing;
                updateShooterSpeed();
            }
        }

        void updateShooterSpeed() {
            double ticks = 0;
            if (isShooting) {
                ticks = targetRPM * constants.SHOOTER_MOTOR_TICK_COUNT / 60.0;
            }
            motorShooter1.setVelocity(ticks);
            motorShooter2.setVelocity(ticks);
        }

        double getShooter1RPM() {
            return (motorShooter1.getVelocity() / constants.SHOOTER_MOTOR_TICK_COUNT) * 60.0;
        }

        double getShooter2RPM() {
            return (motorShooter2.getVelocity() / constants.SHOOTER_MOTOR_TICK_COUNT) * 60.0;
        }

        boolean isAtTargetSpeed() {
            if (!isShooting || targetRPM == 0) return false;
            double rpm = getShooter2RPM();
            return rpm >= (targetRPM - errorRange) && rpm <= (targetRPM + errorRange);
        }

        void intakeStart() { motorIntake.setPower(constants.INTAKE_FORWARD_POWER); }
        void intakeReverse() { motorIntake.setPower(constants.INTAKE_REVERSE_POWER); }
        void intakeStop() { motorIntake.setPower(0); }
        void loadStart() { motorLoad.setPower(constants.LOAD_FORWARD_POWER); }
        void loadReverse() { motorLoad.setPower(constants.LOAD_REVERSE_POWER); }
        void loadStop() { motorLoad.setPower(0); }
        void stopShooter() {
            isShooting = false;
            updateShooterSpeed();
        }
        void stopAll() { intakeStop(); loadStop(); stopShooter(); }

        String getIntakeStatus() {
            double p = motorIntake.getPower();
            if (p > 0) return "吸入";
            if (p < 0) return "吐出";
            return "停止";
        }

        String getLoadStatus() {
            double p = motorLoad.getPower();
            if (p > 0) return "装填";
            if (p < 0) return "退回";
            return "停止";
        }
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

        void resetYaw() {
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

    // ========== 内部类：输入处理 (已修改) ==========
    class ControlInput {
        boolean lastY = false, lastBumper = false;
        boolean lastX = false; // 新增：X键状态
        boolean lastOptions = false; // 新增：Back键状态
        boolean lastUp = false, lastDown = false, lastLeft = false, lastRight = false;

        double getDriveFB() { return -gamepad1.left_stick_y; }
        double getDriveLR() { return gamepad1.left_stick_x; }
        double getTurn() { return gamepad1.right_stick_x; }

        boolean getA() { return gamepad1.a; }
        boolean getB() { return gamepad1.b; }

        boolean getLeftTrigger() { return gamepad1.left_trigger > 0.1; }
        boolean getLeftBumper() { return gamepad1.left_bumper; }
        boolean getRightTrigger() { return gamepad1.right_trigger > 0.1; }

        boolean getYPressed() {
            boolean current = gamepad1.y;
            boolean result = current && !lastY;
            lastY = current;
            return result;
        }

        // 新增：X 键检测（用于切换有头/无头）
        boolean getXPressed() {
            boolean current = gamepad1.x;
            boolean result = current && !lastX;
            lastX = current;
            return result;
        }

        // 新增：Options/Back 键检测（用于重置 IMU）
        boolean getOptionsPressed() {
            boolean current = gamepad1.back || gamepad1.options; // 兼容不同手柄名称
            boolean result = current && !lastOptions;
            lastOptions = current;
            return result;
        }

        boolean getRightBumperPressed() {
            boolean current = gamepad1.right_bumper;
            boolean result = current && !lastBumper;
            lastBumper = current;
            return result;
        }

        // Dpad Pressed 检测
        boolean getDpadRightPressed() {
            boolean current = gamepad1.dpad_right;
            boolean result = current && !lastRight;
            lastRight = current;
            return result;
        }
        boolean getDpadLeftPressed() {
            boolean current = gamepad1.dpad_left;
            boolean result = current && !lastLeft;
            lastLeft = current;
            return result;
        }
        boolean getDpadUpPressed() {
            boolean current = gamepad1.dpad_up;
            boolean result = current && !lastUp;
            lastUp = current;
            return result;
        }
        boolean getDpadDownPressed() {
            boolean current = gamepad1.dpad_down;
            boolean result = current && !lastDown;
            lastDown = current;
            return result;
        }

        void rumble() {
            // 使用带有强度的 rumble 方法，以确保在 Xbox 手柄上能被明确感知
            // 参数：(左侧马达强度, 右侧马达强度, 持续时间毫秒)
            gamepad1.rumble(1.0, 1.0, constants.RUMBLE_DURATION_MS);
        }
    }

    // ========== 内部类：遥测显示 ==========
    class TelemetryDisplay {
        void displayFull(String runtime, int targetRPM, double s1RPM, double s2RPM,
                         boolean atSpeed, String chassisMode, String intakeStatus, String loadStatus,
                         double heading, boolean autoTurning) {
            telemetry.addLine("========== TeleOp v2.3 ==========");
            telemetry.addData("运行时间", runtime);

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

            // 注意初始化顺序：先 Navigation，再 ChassisDrive (因为Chassis依赖Navigation)
            navigation = new Navigation(imu);
            chassis = new ChassisDrive(fl, fr, bl, br, navigation);
            subsystems = new Subsystems(intake, load, s1, s2);
            controlInput = new ControlInput();
            telemetryDisplay = new TelemetryDisplay();

            navigation.initialize(); // 先初始化IMU
            chassis.initialize();
            subsystems.initialize();

        } catch (Exception e) {
            telemetry.addData("初始化错误", e.getMessage());
            telemetry.update();
            return;
        }

        telemetry.addData("状态", "准备就绪 (v2.3 无头模式集成)");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // 1. 导航与底盘控制
            // 自动转向
            if (controlInput.getRightBumperPressed()) {
                navigation.startAutoTurn(constants.AUTO_TURN_TARGET_RIGHT);
            }

            // Y键：切换 线性/非线性
            if (controlInput.getYPressed()) {
                chassis.toggleDriveMode();
                controlInput.rumble();
            }

            // X键：切换 有头/无头
            if (controlInput.getXPressed()) {
                chassis.toggleCentricMode();
                controlInput.rumble();
            }

            // Back键：重置 IMU 航向 (无头模式校准)
            if (controlInput.getOptionsPressed()) {
                navigation.resetYaw();
                controlInput.rumble();
            }

            double autoTurnPower = navigation.calculateTurnPower();
            chassis.update(
                    controlInput.getDriveFB(),
                    controlInput.getDriveLR(),
                    controlInput.getTurn(),
                    navigation.autoTurning,
                    autoTurnPower
            );

            // 2. 拾取系统
            if (controlInput.getA()) {
                subsystems.intakeStart();
            } else if (controlInput.getB()) {
                subsystems.intakeReverse();
            } else {
                subsystems.intakeStop();
            }

            // 3. 装填系统
            if (controlInput.getLeftBumper()) {
                subsystems.loadReverse();
            } else if (controlInput.getLeftTrigger()) {
                subsystems.loadStart();
            } else {
                subsystems.loadStop();
            }

            // 4. 发射系统 - 转速设置
            boolean rpmChanged = false;
            if (controlInput.getDpadRightPressed()) {
                subsystems.setShooterRPM(constants.SHOOTER_RPM_LONG_RANGE);
                rpmChanged = true;
            } else if (controlInput.getDpadLeftPressed()) {
                subsystems.setShooterRPM(constants.SHOOTER_RPM_TRIANGLE_SIDE);
                rpmChanged = true;
            } else if (controlInput.getDpadDownPressed()) {
                subsystems.setShooterRPM(constants.SHOOTER_RPM_TRIANGLE_BASE);
                rpmChanged = true;
            } else if (controlInput.getDpadUpPressed()) {
                subsystems.setShooterRPM(constants.SHOOTER_RPM_TRIANGLE_TOP);
                rpmChanged = true;
            }

            if (rpmChanged) {
                controlInput.rumble();
            }

            // 5. 发射系统 - 激活控制
            subsystems.setShootingState(controlInput.getRightTrigger());

            // 6. 遥测显示
            telemetryDisplay.displayFull(
                    String.format("%.1f s", runtime.seconds()),
                    subsystems.targetRPM,
                    subsystems.getShooter1RPM(),
                    subsystems.getShooter2RPM(),
                    subsystems.isAtTargetSpeed(),
                    chassis.getDriveModeName(),
                    subsystems.getIntakeStatus(),
                    subsystems.getLoadStatus(),
                    navigation.getHeading(),
                    navigation.autoTurning
            );
            telemetry.update();
        }

        chassis.stop();
        subsystems.stopAll();
    }
}
