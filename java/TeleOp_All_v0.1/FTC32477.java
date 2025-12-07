package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
* FTC32477 麦克纳姆轮小车控制程序 - Xbox手柄版
* 适用于四个麦克纳姆轮的机器人
* 
* 控制方式：
* - 左摇杆：控制各方向平移（前后左右移动）
* - 右摇杆X轴：原地转向（倍率80%）
* 
* 控制按键绑定：
* - LT（左扳机）：装填模块正向（load）
* - LB（左肩键）：装填模块反向（load）
* - A键：拾取模块正向（intake）
* - B键：拾取模块反向（intake）
* - RT（右扳机）：发射模块（shooter，s1/s2）
* - D-Pad上/下：调节发射模块目标转速（RPM微调，步长50）
* - Y键：切换线性/非线性映射模式（仅影响左摇杆的前后左右移动）
* 
* 预留但未绑定具体按键的功能：
* - 发射模块的高级预设转速（如“超远”“三角腰”等场地位置）——未绑定具体按键，仅保留D-Pad上下微调。
* - 自动转向（如IMU辅助的精准角度旋转）——未绑定具体按键。
* - 其他底盘/子系统扩展功能可在此基础上继续添加。
* 
* 注意：后两个轮子物理安装方向不同，在运动学公式中直接补偿。
* 
* @author FTC32477
* @version 1.2 - 按键功能注释增强版
 */
@TeleOp(name = "FTC32477 Mecanum Drive", group = "TeleOp")
public class FTC32477 extends LinearOpMode {

    // 定义四个麦克纳姆轮电机
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    
    // 拾取模块（intake）电机
    private DcMotor intake;

    // 装填模块（load）电机
    private DcMotor load;

    // 发射模块（shooter）双电机：s1、s2（闭环控制）
    private DcMotorEx s1;
    private DcMotorEx s2;

    // 发射模块速度闭环控制参数
    public static final double MOTOR_TICK_COUNT = 28;
    public static double P = 135, I = 0, D = 80, F = 14;
    public static double TARGET_RPM = 1500;
    public static int ErrorRange = 50;
    private static final double SHOOTER_SPEED_STEP = 50; // RPM步长
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean lastYState = false;
    private boolean state = false;

    // 非线性映射模式控制
    private boolean useNonlinearMapping = true; // 默认使用非线性映射
    private boolean prevYButton = false;

    // 电机功率限制（安全考虑）
    private static final double MAX_POWER = 1.0;
    
    // 死区设置（避免摇杆漂移）
    private static final double DEADZONE = 0.1;

    @Override
    public void runOpMode() {
        // 初始化硬件
        initializeHardware();
        
        // 等待比赛开始
        telemetry.addData("状态", "等待开始...");
        telemetry.update();
        waitForStart();

        // 主循环
        while (opModeIsActive()) {
            // 处理A键切换非线性映射模式
            handleMappingModeToggle();
            
            // Xbox手柄控制输入
            // 左摇杆：控制各方向平移（前后左右）
            double drive = -gamepad1.left_stick_y;    // 前后移动
            double strafe = gamepad1.left_stick_x;    // 左右平移
            // 右摇杆：原地转向（降低倍率）
            double turn = gamepad1.right_stick_x * 0.8;  // 原地旋转，倍率80%

            // 应用非线性映射（如果启用）
            if (useNonlinearMapping) {
                drive = applyNonlinearMapping(drive);
                strafe = applyNonlinearMapping(strafe);
            }

            // 麦克纳姆轮运动学计算
            double[] wheelPowers = calculateMecanumDrive(drive, strafe, turn);
            
            // 设置电机功率
            setMotorPowers(wheelPowers);
            // 处理子系统按键/扳机控制（intake/load/shooter）
            handleSubsystemControls();

            // 处理发射模块速度调节（D-Pad 上/下）
            handleShooterSpeedAdjustment();

            // 显示调试信息
            displayTelemetry(drive, strafe, turn, wheelPowers);
            
            // 更新遥测数据
            telemetry.update();
        }
    }

    /**
     * 初始化硬件设备
     */
    private void initializeHardware() {
        telemetry.addData("状态", "正在初始化硬件...");
        telemetry.update();

        try {
            // 初始化四个麦克纳姆轮电机
            frontLeft = hardwareMap.get(DcMotor.class, "lf");
            frontRight = hardwareMap.get(DcMotor.class, "rf");
            backLeft = hardwareMap.get(DcMotor.class, "lb");
            backRight = hardwareMap.get(DcMotor.class, "rb");
            
            // 初始化拾取模块（intake）电机
            intake = hardwareMap.get(DcMotor.class, "intake");

            // 初始化装填模块（load）电机
            load = hardwareMap.get(DcMotor.class, "load");

            // 初始化发射模块（shooter）电机：s1、s2（闭环控制）
            s1 = hardwareMap.get(DcMotorEx.class, "s1");
            s2 = hardwareMap.get(DcMotorEx.class, "s2");

            // 设置电机方向
            frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.FORWARD);
            
            // 设置拾取模块（intake）电机方向（可根据实际需要调整）
            intake.setDirection(DcMotorSimple.Direction.FORWARD);

            // 设置装填模块（load）电机方向（反转）
            load.setDirection(DcMotorSimple.Direction.REVERSE);

            // 设置发射模块（shooter）电机s1、s2方向（闭环控制建议与EXP一致）
            s1.setDirection(DcMotorSimple.Direction.REVERSE);
            s2.setDirection(DcMotorSimple.Direction.FORWARD);

            // 设置所有电机运行模式
            setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            load.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            s1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            s2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // 设置发射模块电机零功率行为为锁定（BRAKE），避免怠速旋转
            s1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            s2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // 停止所有电机
            stopAllMotors();
            intake.setPower(0);
            load.setPower(0);
            s1.setPower(0);
            s2.setPower(0);

            telemetry.addData("状态", "硬件初始化完成");
            telemetry.addData("提示", "如果仍有错误，请检查Robot Configuration");
            telemetry.update();

        } catch (Exception e) {
            telemetry.addData("错误", "硬件初始化失败: " + e.getMessage());
            telemetry.addData("解决方案", "请检查Robot Configuration中的设备名称");
            telemetry.addData("常见名称", "frontLeft, frontRight, backLeft, backRight");
            telemetry.addData("或", "lf, rf, lb, rb");
            telemetry.addData("或", "mm, rf, lb, rb (根据错误信息)");
            telemetry.update();
        }
    }

    /**
     * 计算麦克纳姆轮运动学（适配Xbox手柄）
     * @param drive 前后移动 (-1 到 1) - 左摇杆Y轴
     * @param strafe 左右平移 (-1 到 1) - 左摇杆X轴
     * @param turn 原地旋转 (-1 到 1) - 右摇杆X轴
     * @return 四个轮子的功率数组 [FL, FR, BL, BR]
     */
    private double[] calculateMecanumDrive(double drive, double strafe, double turn) {

        // 应用死区
        drive = applyDeadzone(drive);
        strafe = applyDeadzone(strafe);
        turn = applyDeadzone(turn);

        double leftFrontPower  = drive + strafe + turn;
        double rightFrontPower = drive - strafe - turn;
        double leftBackPower   = drive - strafe + turn;
        double rightBackPower  = drive + strafe - turn;

        // 归一化功率，确保不超过1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // 应用最大功率限制
        leftFrontPower  *= MAX_POWER;
        rightFrontPower *= MAX_POWER;
        leftBackPower   *= MAX_POWER;
        rightBackPower  *= MAX_POWER;

        return new double[]{leftFrontPower, rightFrontPower, leftBackPower, rightBackPower};
    }

    /**
     * 处理发射模块（shooter）速度调节：
     * D-Pad 上：+0.05；D-Pad 下：-0.05；单次点击生效（边沿检测）
     */
    private void handleShooterSpeedAdjustment() {
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;

        if (up && !prevDpadUp) {
            TARGET_RPM = Range.clip(TARGET_RPM + SHOOTER_SPEED_STEP, 0.0, 1.0);
        }

        if (down && !prevDpadDown) {
            TARGET_RPM = Range.clip(TARGET_RPM - SHOOTER_SPEED_STEP, 0.0, 1.0);
        }

        prevDpadUp = up;
        prevDpadDown = down;
    }

    /**
     * 处理A键切换非线性映射模式（边沿检测）
     */
    private void handleMappingModeToggle() {
        boolean yPressed = gamepad1.y;
        if (yPressed && !prevYButton) {
            useNonlinearMapping = !useNonlinearMapping;
        }
        prevYButton = yPressed;
    }

    /**
     * 应用非线性映射：使用平方映射，小幅度时更精确，最大值时保持为1
     * @param value 输入值 (-1 到 1)
     * @return 映射后的值
     */
    private double applyNonlinearMapping(double value) {
        if (value == 0.0) {
            return 0.0;
        }
        // 保持符号，应用平方映射
        double sign = value > 0 ? 1.0 : -1.0;
        return sign * value * value;
    }

    /**
     * 应用死区限制
     */
    private double applyDeadzone(double value) {
        if (Math.abs(value) < DEADZONE) {
            return 0.0;
        }
        return value;
    }

    /**
     * 设置所有电机功率
     */
    private void setMotorPowers(double[] wheelPowers) {
        frontLeft.setPower(wheelPowers[0]);
        frontRight.setPower(wheelPowers[1]);
        backLeft.setPower(wheelPowers[2]);
        backRight.setPower(wheelPowers[3]);
    }

    /**
     * 设置所有电机运行模式
     */
    private void setMotorModes(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    /**
     * 处理子系统按键/扳机控制：
     * - 左侧扳机 (LT): `load` 电机正转
     * - 右侧扳机 (RT): `s1` 正转，`s2` 反转
     * - A 键: `intake` 正转
     * - B 键: `intake` 反转
     */
    private void handleSubsystemControls() {
        // Load 控制（左扳机）
        if (gamepad1.left_trigger > DEADZONE) {
            load.setPower(1.0);
        } else {
            load.setPower(0);
        }

        // Shooter 控制（右扳机）：s1 正转，s2 反转
        if (gamepad1.right_trigger > DEADZONE) {
            s1.setPower(1.0);
            s2.setPower(-1.0);
        } else {
            s1.setPower(0);
            s2.setPower(0);
        }

        // Intake 控制（A/B）
        if (gamepad1.a) {
            intake.setPower(1.0);
        } else if (gamepad1.b) {
            intake.setPower(-1.0);
        } else {
            intake.setPower(0);
        }
    }

    /**
     * 停止所有电机
     */
    private void stopAllMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    /**
     * 显示遥测信息
     */
    private void displayTelemetry(double drive, double strafe, double turn, double[] wheelPowers) {
        double currentRPM = (s1.getVelocity() / MOTOR_TICK_COUNT) * 60;
        boolean isShooting = gamepad1.right_trigger > DEADZONE;
        boolean isAtTargetSpeed = (Math.abs(currentRPM - TARGET_RPM) < ErrorRange);

        telemetry.addData("=== 控制模式 ===", "");
        telemetry.addData("映射模式", useNonlinearMapping ? "非线性(平方)" : "线性");
        
        telemetry.addData("=== 发射模块 ===", "");
        telemetry.addData("状态", isShooting ? (isAtTargetSpeed ? "发射中" : "提速中") : "关闭");
        telemetry.addData("目标 RPM", TARGET_RPM);
        telemetry.addData("当前 RPM", "%.2f", currentRPM);

        telemetry.addData("=== 装填模块 ===", "");
        telemetry.addData("load功率", "%.2f", load.getPower());

        telemetry.addData("=== 拾取模块 ===", "");
        telemetry.addData("intake功率", "%.2f", intake.getPower());

        telemetry.addData("=== 底盘电机 ===", "");
        telemetry.addData("功率", "FL:%.2f FR:%.2f BL:%.2f BR:%.2f", 
                         wheelPowers[0], wheelPowers[1], wheelPowers[2], wheelPowers[3]);
        
        telemetry.addData("--- 按键说明 ---", "");
        telemetry.addData("RT", "智能发射");
        telemetry.addData("D-Pad 上/下", "RPM +/-");
        telemetry.addData("Y", "切换映射模式");
    }
}
