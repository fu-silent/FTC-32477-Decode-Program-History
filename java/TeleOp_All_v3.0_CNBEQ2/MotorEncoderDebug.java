package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

// 这是一个 TeleOp 程序，用于手动调试
@TeleOp(name="DEBUG - Motor and Encoder Test", group="Debug")
public class MotorEncoderDebug extends LinearOpMode {

    // 声明底盘电机，使用 ChassisDrive.java 中的映射名称
    private DcMotor fl, fr, bl, br;
    private ElapsedTime runtime = new ElapsedTime();
    private final double TEST_POWER = 0.5; // 用于测试的固定功率

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initializing Motors...");
        telemetry.update();

        // 1. 映射电机 (使用 ChassisDrive.java 中的映射字符串)
        try {
            fl = hardwareMap.get(DcMotor.class, "lf"); // 前左
            fr = hardwareMap.get(DcMotor.class, "rf"); // 前右
            bl = hardwareMap.get(DcMotor.class, "lb"); // 后左
            br = hardwareMap.get(DcMotor.class, "rb"); // 后右
        } catch (Exception e) {
            telemetry.addData("ERROR", "Motor mapping failed: %s", e.getMessage());
            telemetry.update();
            sleep(5000);
            return;
        }

        // 2. 核心设置：重置编码器并设置为 RUN_WITHOUT_ENCODER
        // RUN_WITHOUT_ENCODER 允许我们直接设置功率，并读取编码器原始值
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 3. 设置方向 (使用 ChassisDrive.java 中的初始方向，便于对照)
        // FL.setDirection(DcMotorSimple.Direction.REVERSE);  // 前左
        // FR.setDirection(DcMotorSimple.Direction.FORWARD);   // 前右
        // BL.setDirection(DcMotorSimple.Direction.REVERSE);  // 后左
        // BR.setDirection(DcMotorSimple.Direction.FORWARD);   // 后右
        // 注：这里可以先沿用 ChassisDrive 的设置，然后在测试中决定是否需要修改。
        // 为了使测试结果独立于 ChassisDrive，这里我们默认使用正向，便于测试极性。
        fl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialization Complete. Press START.");
        telemetry.addData("Control", "Use Gamepad 1 Buttons to test.");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // --- 4. 电机控制逻辑 ---
            double fl_p = 0, fr_p = 0, bl_p = 0, br_p = 0;

            // 前左 (FL) - A/X 键
            if (gamepad1.a) fl_p = TEST_POWER; // 正向
            else if (gamepad1.x) fl_p = -TEST_POWER; // 反向

            // 前右 (FR) - B/Y 键
            if (gamepad1.b) fr_p = TEST_POWER; // 正向
            else if (gamepad1.y) fr_p = -TEST_POWER; // 反向

            // 后左 (BL) - 左缓冲键 (LB) / 左摇杆按钮 (LSB)
            if (gamepad1.left_bumper) bl_p = TEST_POWER; // 正向
            else if (gamepad1.left_stick_button) bl_p = -TEST_POWER; // 反向

            // 后右 (BR) - 右缓冲键 (RB) / 右摇杆按钮 (RSB)
            if (gamepad1.right_bumper) br_p = TEST_POWER; // 正向
            else if (gamepad1.right_stick_button) br_p = -TEST_POWER; // 反向

            // 应用功率
            fl.setPower(fl_p);
            fr.setPower(fr_p);
            bl.setPower(bl_p);
            br.setPower(br_p);

            // --- 5. 遥测信息 ---
            telemetry.addData("Time", "%.1f s", runtime.seconds());
            telemetry.addLine("--------------------------------");
            telemetry.addLine("电机状态 | 功率 | 编码器读数 | 按钮");
            telemetry.addLine("--------------------------------");

            // 前左
            telemetry.addData("FL (lf)", "%.1f | %d | A/X", fl_p, fl.getCurrentPosition());

            // 前右
            telemetry.addData("FR (rf)", "%.1f | %d | B/Y", fr_p, fr.getCurrentPosition());

            // 后左
            telemetry.addData("BL (lb)", "%.1f | %d | LB/LSB", bl_p, bl.getCurrentPosition());

            // 后右
            telemetry.addData("BR (rb)", "%.1f | %d | RB/RSB", br_p, br.getCurrentPosition());

            telemetry.update();
        }
    }
}