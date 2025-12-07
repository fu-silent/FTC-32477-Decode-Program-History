package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * FTC32477 机器人 TeleOp 远程控制程序（单文件集成版 v1.0.0）
 * 
 * 赛季主题：Decode
 * 
 * 设计理念：
 * 本程序采用内部类模式，将所有模块整合为一个文件
 * 保留模块化的逻辑结构，减少内存占用和编译时间
 * 
 * 内容结构：
 * 1. 主类 TeleOp_All_1_0_0 - 程序入口和控制循环
 * 2. 内部类 Constants - 全局常数管理
 * 3. 内部类 ChassisDrive - 底盘驱动系统
 * 4. 内部类 Subsystems - 子系统管理
 * 5. 内部类 ControlInput - 控制输入处理
 * 6. 内部类 TelemetryDisplay - 遥测显示
 * 
 * 功能完整性：
 * ✓ 底盘：麦克纳姆轮全向控制
 * ✓ 拾取、装填、发射三个子系统
 * ✓ 发射功率调节和反馈
 * ✓ 线性/非线性映射切换
 * ✓ 完整的遥测显示
 * 
 * @author FTC32477
 * @version 1.0.0 - 单文件集成版
 */
@TeleOp(name = "TeleOp_All_1_0 - Integrated", group = "TeleOp")
public class TeleOp_All_1_0 extends LinearOpMode {

    // ==================== 模块实例 ====================
    private Constants constants;
    private ChassisDrive chassis;
    private Subsystems subsystems;
    private ControlInput controlInput;
    private TelemetryDisplay telemetry_mgr;

    @Override
    public void runOpMode() {
        // 初始化所有系统
        if (!initializeAllSystems()) {
            telemetry_mgr.displayError("系统初始化失败，请检查硬件连接");
            return;
        }

        telemetry_mgr.displayWaitingForStart();
        waitForStart();

        // 主控制循环
        while (opModeIsActive()) {
            controlInput.updateInputs();
            chassis.update(controlInput);
            subsystems.update(controlInput);
            telemetry_mgr.display(controlInput, chassis, subsystems);
        }

        // 停止所有系统
        chassis.stop();
        subsystems.stopAll();
    }

    private boolean initializeAllSystems() {
        try {
            telemetry.addData("状态", "正在初始化系统...");
            telemetry.update();

            // 初始化模块
            constants = new Constants();
            chassis = new ChassisDrive();
            subsystems = new Subsystems();
            controlInput = new ControlInput(gamepad1);
            telemetry_mgr = new TelemetryDisplay(this);

            // 初始化硬件
            if (!chassis.initialize(hardwareMap) || !subsystems.initialize(hardwareMap)) {
                telemetry.addData("错误", "硬件初始化失败");
                telemetry.update();
                return false;
            }

            telemetry_mgr.displayInitStatus(
                chassis.getStatus(),
                subsystems.getStatus()
            );
            return true;

        } catch (Exception e) {
            telemetry.addData("异常", e.getMessage());
            telemetry.update();
            return false;
        }
    }

    // ==================== 内部类：常数管理 ====================
    
    private class Constants {
        // 硬件映射名称
        final String CHASSIS_FL = "lf";
        final String CHASSIS_FR = "rf";
        final String CHASSIS_BL = "lb";
        final String CHASSIS_BR = "rb";

        final String SUBSYS_INTAKE = "intake";
        final String SUBSYS_LOAD = "load";
        final String SUBSYS_S1 = "s1";
        final String SUBSYS_S2 = "s2";

        // 底盘参数
        final double DEADZONE = 0.1;
        final double TURN_SCALE = 0.8;
        final double MAX_POWER = 1.0;
        final boolean NONLINEAR_DEFAULT = true;

        // 子系统参数
        final double TRIGGER_DEADZONE = 0.1;
        final double INTAKE_POWER = 1.0;
        final double LOAD_POWER = 1.0;
        final double SHOOTER_S1_POWER = 1.0;
        final double SHOOTER_S2_POWER = -1.0;

        // 发射参数（功率控制）
        final double SHOOTER_POWER_DEFAULT = 0.8;
        final double SHOOTER_POWER_STEP = 0.05;          // D-Pad 上下步长
        final double SHOOTER_POWER_STEP_LR = 0.02;       // D-Pad 左右步长
        final double SHOOTER_POWER_MIN = 0.0;
        final double SHOOTER_POWER_MAX = 1.0;
    }

    // ==================== 内部类：底盘驱动系统 ====================

    private class ChassisDrive {
        private DcMotor motorFL, motorFR, motorBL, motorBR;
        private boolean initialized = false;

        boolean initialize(com.qualcomm.robotcore.hardware.HardwareMap hw) {
            try {
                motorFL = hw.get(DcMotor.class, constants.CHASSIS_FL);
                motorFR = hw.get(DcMotor.class, constants.CHASSIS_FR);
                motorBL = hw.get(DcMotor.class, constants.CHASSIS_BL);
                motorBR = hw.get(DcMotor.class, constants.CHASSIS_BR);

                motorFL.setDirection(DcMotor.Direction.FORWARD);
                motorFR.setDirection(DcMotor.Direction.REVERSE);
                motorBL.setDirection(DcMotor.Direction.REVERSE);
                motorBR.setDirection(DcMotor.Direction.FORWARD);

                motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    // 设置零功率行为为制动，确保摇杆回中时车辆快速刹停
                    motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    stop();
                initialized = true;
                return true;
            } catch (Exception e) {
                return false;
            }
        }

        void update(ControlInput input) {
            double drive = applyDeadzone(input.getDriveFB());
            double strafe = applyDeadzone(input.getStrafeLR());
            double turn = applyDeadzone(input.getRotate());

            // 如果所有输入为0，则立即停止（刹停）
            if (drive == 0.0 && strafe == 0.0 && turn == 0.0) {
                motorFL.setPower(0);
                motorFR.setPower(0);
                motorBL.setPower(0);
                motorBR.setPower(0);
                return;
            }

            if (input.isNonlinearMode()) {
                drive = applyNonlinear(drive);
                strafe = applyNonlinear(strafe);
            }

            // 麦克纳姆轮公式
            double pFL = drive + strafe + turn;
            double pFR = drive - strafe - turn;
            double pBL = drive + strafe - turn;
            double pBR = drive - strafe + turn;

            // 归一化
            double max = Math.max(
                Math.max(Math.abs(pFL), Math.abs(pFR)),
                Math.max(Math.abs(pBL), Math.abs(pBR))
            );

            if (max > 1.0) {
                pFL /= max;
                pFR /= max;
                pBL /= max;
                pBR /= max;
            }

            motorFL.setPower(pFL * constants.MAX_POWER);
            motorFR.setPower(pFR * constants.MAX_POWER);
            motorBL.setPower(pBL * constants.MAX_POWER);
            motorBR.setPower(pBR * constants.MAX_POWER);
        }

        private double applyDeadzone(double value) {
            return Math.abs(value) < constants.DEADZONE ? 0.0 : value;
        }

        private double applyNonlinear(double value) {
            if (value == 0.0) return 0.0;
            double sign = value > 0 ? 1.0 : -1.0;
            return sign * value * value;
        }

        void stop() {
            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBL.setPower(0);
            motorBR.setPower(0);
        }

        double[] getPowers() {
            return new double[]{motorFL.getPower(), motorFR.getPower(), motorBL.getPower(), motorBR.getPower()};
        }

        String getStatus() {
            return initialized ? "底盘就绪" : "底盘故障";
        }
    }

    // ==================== 内部类：子系统管理 ====================

    private class Subsystems {
        private DcMotor motorIntake, motorLoad;
        private DcMotorEx motorS1, motorS2;
        private double shooterPower;
        private boolean initialized = false;

        boolean initialize(com.qualcomm.robotcore.hardware.HardwareMap hw) {
            try {
                motorIntake = hw.get(DcMotor.class, constants.SUBSYS_INTAKE);
                motorLoad = hw.get(DcMotor.class, constants.SUBSYS_LOAD);
                motorS1 = hw.get(DcMotorEx.class, constants.SUBSYS_S1);
                motorS2 = hw.get(DcMotorEx.class, constants.SUBSYS_S2);

                motorIntake.setDirection(DcMotor.Direction.FORWARD);
                motorLoad.setDirection(DcMotor.Direction.REVERSE);
                motorS1.setDirection(DcMotor.Direction.FORWARD);
                motorS2.setDirection(DcMotor.Direction.FORWARD);

                motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorLoad.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorS1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorS2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                motorS1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorS2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                shooterPower = constants.SHOOTER_POWER_DEFAULT;
                stopAll();
                initialized = true;
                return true;
            } catch (Exception e) {
                return false;
            }
        }

        void update(ControlInput input) {
            // 拾取控制
            if (input.isIntakeForward()) {
                motorIntake.setPower(constants.INTAKE_POWER);
            } else if (input.isIntakeReverse()) {
                motorIntake.setPower(-constants.INTAKE_POWER);
            } else {
                motorIntake.setPower(0);
            }

            // 装填控制
            if (input.isLoadReverseRequested()) {
                motorLoad.setPower(-constants.LOAD_POWER);
            } else if (input.isLoadRequested()) {
                motorLoad.setPower(constants.LOAD_POWER);
            } else {
                motorLoad.setPower(0);
            }

            // 发射（功率控制）
            if (input.isShooterRequested()) {
                motorS1.setPower(shooterPower);
                motorS2.setPower(-shooterPower);
            } else {
                motorS1.setPower(0);
                motorS2.setPower(0);
            }

            // 功率调节（D-Pad）
            if (input.isPowerUpRequested()) {
                shooterPower = Math.min(constants.SHOOTER_POWER_MAX, shooterPower + constants.SHOOTER_POWER_STEP);
            }
            if (input.isPowerDownRequested()) {
                shooterPower = Math.max(constants.SHOOTER_POWER_MIN, shooterPower - constants.SHOOTER_POWER_STEP);
            }
            if (input.isPowerRightRequested()) {
                shooterPower = Math.min(constants.SHOOTER_POWER_MAX, shooterPower + constants.SHOOTER_POWER_STEP_LR);
            }
            if (input.isPowerLeftRequested()) {
                shooterPower = Math.max(constants.SHOOTER_POWER_MIN, shooterPower - constants.SHOOTER_POWER_STEP_LR);
            }
        }

        void stopAll() {
            motorIntake.setPower(0);
            motorLoad.setPower(0);
            motorS1.setPower(0);
            motorS2.setPower(0);
        }

        double getIntakePower() {
            return motorIntake.getPower();
        }

        double getLoadPower() {
            return motorLoad.getPower();
        }

        double getShooterPower() {
            return shooterPower;
        }

        double getS1Power() {
            return motorS1.getPower();
        }

        double getS2Power() {
            return motorS2.getPower();
        }

        String getStatus() {
            return initialized ? "子系统就绪" : "子系统故障";
        }
    }

    // ==================== 内部类：控制输入处理 ====================
    private class ControlInput {
        private Gamepad gamepad;
        private boolean modeNonlinear;
        private boolean prevY = false, prevUp = false, prevDown = false, prevLeft = false, prevRight = false;

        ControlInput(Gamepad gp) {
            gamepad = gp;
            modeNonlinear = constants.NONLINEAR_DEFAULT;
        }

        void updateInputs() {
            // 模式切换（边沿检测）
            if (gamepad.y && !prevY) {
                modeNonlinear = !modeNonlinear;
            }
            prevY = gamepad.y;
        }

        double getDriveFB() {
            return -gamepad.left_stick_y;
        }

        double getStrafeLR() {
            return gamepad.right_stick_x;
        }

        double getRotate() {
            return gamepad.left_stick_x * constants.TURN_SCALE;
        }

        boolean isNonlinearMode() {
            return modeNonlinear;
        }

        boolean isIntakeForward() {
            return gamepad.a;
        }

        boolean isIntakeReverse() {
            return gamepad.b;
        }

        boolean isLoadRequested() {
            return gamepad.left_trigger > constants.TRIGGER_DEADZONE;
        }

        boolean isLoadReverseRequested() {
            return gamepad.left_bumper;
        }

        boolean isShooterRequested() {
            return gamepad.right_trigger > constants.TRIGGER_DEADZONE;
        }

        boolean isPowerUpRequested() {
            boolean current = gamepad.dpad_up;
            boolean result = current && !prevUp;
            prevUp = current;
            return result;
        }

        boolean isPowerDownRequested() {
            boolean current = gamepad.dpad_down;
            boolean result = current && !prevDown;
            prevDown = current;
            return result;
        }

        boolean isPowerLeftRequested() {
            boolean current = gamepad.dpad_left;
            boolean result = current && !prevLeft;
            prevLeft = current;
            return result;
        }

        boolean isPowerRightRequested() {
            boolean current = gamepad.dpad_right;
            boolean result = current && !prevRight;
            prevRight = current;
            return result;
        }
    }

    // ==================== 内部类：遥测显示 ====================

    private class TelemetryDisplay {
        private LinearOpMode opMode;

        TelemetryDisplay(LinearOpMode op) {
            opMode = op;
        }

        void displayInitStatus(String chassisStatus, String subsysStatus) {
            opMode.telemetry.addData("=== 初始化状态 ===", "");
            opMode.telemetry.addData("底盘", chassisStatus);
            opMode.telemetry.addData("子系统", subsysStatus);
            opMode.telemetry.update();
        }

        void displayWaitingForStart() {
            opMode.telemetry.addData("状态", "等待开始...");
            opMode.telemetry.addData("提示", "按START按钮开始比赛");
            opMode.telemetry.update();
        }

        void display(ControlInput input, ChassisDrive chassis, Subsystems subsys) {
            double[] powers = chassis.getPowers();

            // 底盘信息
            opMode.telemetry.addData("=== 底盘 ===", "");
            opMode.telemetry.addData("模式", input.isNonlinearMode() ? "非线性" : "线性");
            opMode.telemetry.addData("电机功率(FL/FR/BL/BR)", 
                String.format("%.2f/%.2f/%.2f/%.2f", 
                    powers[0], powers[1], powers[2], powers[3]));

            // 发射信息（功率控制）
            opMode.telemetry.addData("=== 发射模块 ===", "");
            String shooterState = input.isShooterRequested() ? "发射中" : "关闭";
            opMode.telemetry.addData("状态", shooterState);
            opMode.telemetry.addData("发射功率(Shooter)", String.format("%.2f", subsys.getShooterPower()));
            opMode.telemetry.addData("S1/S2 功率", String.format("%.2f / %.2f", subsys.getS1Power(), subsys.getS2Power()));

            // 其他系统
            opMode.telemetry.addData("=== 装填 ===", "");
            opMode.telemetry.addData("功率", String.format("%.2f", subsys.getLoadPower()));

            opMode.telemetry.addData("=== 拾取 ===", "");
            opMode.telemetry.addData("功率", String.format("%.2f", subsys.getIntakePower()));

            // 按键说明
            opMode.telemetry.addData("=== 按键 ===", "");
            opMode.telemetry.addData("A/B", "拾取 正/反");
            opMode.telemetry.addData("LT/LB", "装填 正/反");
            opMode.telemetry.addData("RT", "发射");
            opMode.telemetry.addData("D-Pad↑↓", "发射功率 +/-");
            opMode.telemetry.addData("Y", "模式切换");

            opMode.telemetry.update();
        }

        void displayError(String msg) {
            opMode.telemetry.addData("错误", msg);
            opMode.telemetry.update();
        }
    }
}
