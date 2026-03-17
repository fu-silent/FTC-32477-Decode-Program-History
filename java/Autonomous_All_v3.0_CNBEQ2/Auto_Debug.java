package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Auto_Debug", group = "Debug")
public class Auto_Debug extends LinearOpMode {
    RobotHardware robot = new RobotHardware();

    // 自动程序专用参数
    static final double MAX_VEL_TICKS = 2500;
    static final double KP_DRIVE = 0.005;
    static final double KP_TURN = 0.045;
    static final double KP_CORRECTION = 0.06;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // 1. 配置自动程序专用的底盘 PIDF（不影响手动）
        PIDFCoefficients chassisPIDF = new PIDFCoefficients(15, 3, 0.5, 12);
        robot.lf.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, chassisPIDF);
        robot.rf.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, chassisPIDF);
        robot.lb.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, chassisPIDF);
        robot.rb.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, chassisPIDF);

        setChassisRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // 2. 配置自动程序专用的发射器 PIDF
        PIDFCoefficients shooterPIDF = new PIDFCoefficients(250, 0, 100, 14);
        robot.s1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        robot.s2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);

        waitForStart();

        // --- 动作序列 ---
        // 前进 100mm, 锁定 0 度, 侧向补偿 0.22 (针对 12.7 度偏移)
        // moveStraightVelocity(100, 0, 0.5, 0.22, 2.0);

        // 旋转 14 度瞄准
        // rotateVelocity(14, 0.5, 2.5);

        // 发射 (1625 RPM, 持续 3 秒)
        // shootVelocity(1625, 3.0);

        // rotateVelocity(0, 0.5, 2.0);

        // moveStraightVelocity(-150, 0, 0.5, 0.22, 4.0);

        // strafePID(-600, 0, 0.5, 4.0);

        moveStraightVelocity(-25, 0, 0.5, 0, 4.0);

        sleep(100);
    }

    // --- 自动程序私有控制方法 ---

    public void moveStraightVelocity(double mm, double targetAngle, double speed, double strafeComp, double timeout) {
        int targetTicks = (int)(mm * robot.TICKS_PER_MM);
        resetChassisEncoders();
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < timeout) {
            int currentPos = (Math.abs(robot.lf.getCurrentPosition()) + Math.abs(robot.rf.getCurrentPosition())) / 2;
            double error = targetTicks - currentPos;
            if (Math.abs(error) < 15) break;

            double drive = Range.clip(error * KP_DRIVE, -speed, speed);
            double turn = getHeadingError(targetAngle) * KP_CORRECTION;

            // 使用 Velocity 闭环控制
            setVelocities(drive + strafeComp - turn, drive - strafeComp + turn,
                    drive - strafeComp - turn, drive + strafeComp + turn);
        }
        setVelocities(0,0,0,0);
    }

    public void strafePID(double mm, double targetAngle, double speed, double timeout) {
        // 麦轮平移由于辊子滑动损耗，通常物理位移会偏小，此处乘以 1.15 补偿系数
        int targetTicks = (int)(mm * robot.TICKS_PER_MM * 1.15);
        resetChassisEncoders();
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < timeout) {
            // 平移位移计算公式：(LF - RF - LB + RB) / 4
            int currentPos = (int)((robot.lf.getCurrentPosition() - robot.rf.getCurrentPosition()
                    - robot.lb.getCurrentPosition() + robot.rb.getCurrentPosition()) / 4.0);

            double error = targetTicks - currentPos;

            // 容差判断：距离目标小于 20 ticks 则认为到达
            if (Math.abs(error) < 20) break;

            // 计算平移推力 (PID 比例控制)
            double strafe = Range.clip(error * KP_DRIVE, -speed, speed);

            // 实时航向修正：确保平移过程中机器人不发生旋转
            double turn = getHeadingError(targetAngle) * KP_CORRECTION;

            // 麦轮平移速度分配方案：
            // LF = +strafe, RF = -strafe, LB = -strafe, RB = +strafe
            setVelocities(strafe - turn, -strafe + turn,
                    -strafe - turn, strafe + turn);
        }
        // 完成后停止所有电机
        setVelocities(0,0,0,0);
    }

    /**
     * 在前进的同时控制子系统
     * @param mm 前进距离
     * @param targetAngle 锁定航向
     * @param speed 前进速度
     * @param intakeDelay intake 启动延迟（秒）
     * @param intakeDur   intake 持续时间（秒）
     * @param loadDelay   load 启动延迟（秒）
     * @param loadDur     load 持续时间（秒）
     */
    public void moveStraightWithSubsystems(double mm, double targetAngle, double speed,
                                           double intakeDelay, double intakeDur,
                                           double loadDelay, double loadDur, double timeout) {
        int targetTicks = (int)(mm * robot.TICKS_PER_MM);
        resetChassisEncoders();
        ElapsedTime timer = new ElapsedTime(); // 动作开始的总计时器

        while (opModeIsActive() && timer.seconds() < timeout) {
            // --- 1. 底盘移动逻辑 ---
            int currentPos = (int)((Math.abs(robot.lf.getCurrentPosition()) + Math.abs(robot.rf.getCurrentPosition())) / 2.0);
            double error = targetTicks - currentPos;

            // 基础前进功率
            double drive = Range.clip(error * KP_DRIVE, -speed, speed);
            // 航向修正
            double turn = getHeadingError(targetAngle) * KP_CORRECTION;

            // 如果距离目标足够近，停止移动，但循环可能继续（为了完成子系统动作）
            if (Math.abs(error) < 15) drive = 0;

            // --- 2. 子系统时间轴控制逻辑 ---
            double currentTime = timer.seconds();

            // 控制 Intake
            if (currentTime >= intakeDelay && currentTime < (intakeDelay + intakeDur)) {
                robot.intake.setPower(0.9);
            } else {
                robot.intake.setPower(0);
            }

            // 控制 Load
            if (currentTime >= loadDelay && currentTime < (loadDelay + loadDur)) {
                robot.load.setPower(0.8);
            } else {
                robot.load.setPower(0);
            }

            // --- 3. 执行底盘转速输出 ---
            // 注意：这里我们假设不再需要额外的横向补偿 strafeComp，如有需要可自行添加
            setVelocities(drive - turn, drive + turn, drive - turn, drive + turn);

            // 如果底盘到达且子系统所有动作都结束，提前退出循环
            if (Math.abs(error) < 15 && currentTime > (intakeDelay + intakeDur) && currentTime > (loadDelay + loadDur)) {
                break;
            }
        }
        // 彻底停止
        setVelocities(0,0,0,0);
        robot.intake.setPower(0);
        robot.load.setPower(0);
    }

    public void rotateVelocity(double targetAngle, double speed, double timeout) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < timeout) {
            double error = getHeadingError(targetAngle);
            if (Math.abs(error) < 1.0) break;
            double turn = Range.clip(error * KP_TURN, -speed, speed);
            if (Math.abs(turn) < 0.15) turn = Math.signum(turn) * 0.15;
            setVelocities(-turn, turn, -turn, turn);
        }
        setVelocities(0,0,0,0);
    }

    public void shootVelocity(int rpm, double duration) {
        double ticksPerSec = (rpm * robot.SHOOTER_TICKS) / 60.0;
        robot.s1.setVelocity(ticksPerSec);
        robot.s2.setVelocity(ticksPerSec);
        robot.intake.setPower(0.95);
        sleep(1500); // 等待转速稳定
        robot.load.setPower(0.85);
        sleep((long)(duration * 1000));
        robot.s1.setVelocity(0); robot.s2.setVelocity(0);
        robot.intake.setPower(0); robot.load.setPower(0);
    }

    private void setVelocities(double vLF, double vRF, double vLB, double vRB) {
        robot.lf.setVelocity(vLF * MAX_VEL_TICKS);
        robot.rf.setVelocity(vRF * MAX_VEL_TICKS);
        robot.lb.setVelocity(vLB * MAX_VEL_TICKS);
        robot.rb.setVelocity(vRB * MAX_VEL_TICKS);
    }

    private double getHeadingError(double target) {
        double heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = target - heading;
        while (error > 180) error -= 360;
        while (error <= -180) error += 360;
        return error;
    }

    private void resetChassisEncoders() {
        setChassisRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setChassisRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setChassisRunMode(DcMotor.RunMode mode) {
        robot.lf.setMode(mode); robot.rf.setMode(mode);
        robot.lb.setMode(mode); robot.rb.setMode(mode);
    }
}