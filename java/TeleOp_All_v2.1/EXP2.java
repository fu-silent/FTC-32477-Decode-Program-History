package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class EXP2 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private IMU imu;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor intake = null;
    private DcMotor blender = null;
    private DcMotorEx shooter1 = null;
    private DcMotorEx shooter2 = null;



    private boolean lastYState = false;
    private boolean state = false;
    public static final double MOTOR_TICK_COUNT = 28;
    public static double P = 135, I = 0, D = 10, F = 13.5;
    public static double TARGET_RPM = 0;
    public static int ErrorRange = 50;

    //-----------------------------------------------------------------------
    // 控制参数
    private double targetHeading = 0;
    private boolean isTurningToTarget = false;
    private final double TURN_POWER = 0.5;
    private final double HEADING_THRESHOLD = 2.0; // 角度阈值（度）
    private final double P_TURN_GAIN = 0.1; // 转向比例增益

    // PID控制参数
    private double integralSum = 0;
    private double previousError = 0;
    private long previousTime = 0;
    private final double I_GAIN = 0.000; // 积分增益
    private final double D_GAIN = 0.005; // 微分增益

    // 目标角度常量
    private final double TARGET_ANGLE_LEFT = -45.0;  // 左转45度
    private final double TARGET_ANGLE_RIGHT = 45.0;  // 右转45度

    @Override
    public void runOpMode() {

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "lf");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "lb");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");
        intake  = hardwareMap.get(DcMotor.class, "intake");
        blender = hardwareMap.get(DcMotor.class, "load");
        shooter1 = hardwareMap.get(DcMotorEx.class, "s1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "s2");

        imu = hardwareMap.get(IMU.class,"imu");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setDirection(DcMotor.Direction.FORWARD);
        blender.setDirection(DcMotor.Direction.REVERSE);
        shooter1.setDirection(DcMotor.Direction.FORWARD);
        shooter2.setDirection(DcMotor.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);


        //---------------------------------------------
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        sleep(500);
        telemetry.addData("状态", "IMU偏航角已重置");
        telemetry.update();

        // 初始化PID时间戳
        previousTime = System.nanoTime();

        //----------------------------------------------
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // 检查是否按下右侧肩键触发自动转向45度
            if (gamepad1.right_bumper && !isTurningToTarget) {
                targetHeading = TARGET_ANGLE_RIGHT;
                isTurningToTarget = true;
                // 重置PID状态
                resetPID();
                telemetry.addData("自动转向", "转向 45 度");
            }

            // 如果正在自动转向，执行转向控制
            if (isTurningToTarget) {
                performAutoTurn();
            }
            else{
                double max;

                // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
                double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
                double lateral =  gamepad1.left_stick_x;
                double yaw     =  gamepad1.right_stick_x;

                // 如果正在自动转向，忽略手动转向输入
                if (isTurningToTarget) {
                    yaw = 0;
                }

                // Combine the joystick requests for each axis-motion to determine each wheel's power.
                // Set up a variable for each drive wheel to save the power level for telemetry.
                double leftFrontPower  = axial + lateral + yaw;
                double rightFrontPower = axial - lateral - yaw;
                double leftBackPower   = axial - lateral + yaw;
                double rightBackPower  = axial + lateral - yaw;

                // Normalize the values so no wheel power exceeds 100%
                // This ensures that the robot maintains the desired motion.
                max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightBackPower));

                if (max > 1.0) {
                    leftFrontPower  /= max;
                    rightFrontPower /= max;
                    leftBackPower   /= max;
                    rightBackPower  /= max;
                }

                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);
            }

            // 将目标RPM转换为每秒的编码器刻度数
            double targetTicksPerSecond = TARGET_RPM * MOTOR_TICK_COUNT / 60;
            // 设置电机的目标速度
            shooter1.setVelocity(targetTicksPerSecond);
            shooter2.setVelocity(targetTicksPerSecond);

            if(gamepad1.a){
                state=false;

                intake.setPower(0.8);
                blender.setPower(0);
                TARGET_RPM = 0;
            }

            if(gamepad1.b){
                intake.setPower(-0.8);
                blender.setPower(-0.55);
                shooter1.setPower(0);
                shooter2.setPower(0);
            }
            if(gamepad1.x){
                intake.setPower(0);
                blender.setPower(0);
                TARGET_RPM = 0;
            }


            if(gamepad1.dpad_right){
                TARGET_RPM = 1800;//超远
                ErrorRange = 200;
            }
            if(gamepad1.dpad_left){
                TARGET_RPM = 1400;//三角腰
                ErrorRange = 200;
            }
            if(gamepad1.dpad_down){
                TARGET_RPM = 1200;//三角底部
                ErrorRange = 200;
            }
            if(gamepad1.dpad_up){
                TARGET_RPM = 1650;//三角顶点
                ErrorRange = 200;
            }
            if(gamepad1.right_stick_button){
                TARGET_RPM = 4000;//测试用高速
                ErrorRange = 200;
            }



            double currentVelocityTicks1 = shooter1.getVelocity();
            double currentRPM1 = (currentVelocityTicks1 / MOTOR_TICK_COUNT) * 60;
            double currentVelocityTicks2 = shooter2.getVelocity();
            double currentRPM2 = (currentVelocityTicks2 / MOTOR_TICK_COUNT) * 60;
            boolean volocitycheck1 = false;
            boolean volocitycheck2 = false;
            boolean volocitycheck = false;
            if((TARGET_RPM + ErrorRange)>currentRPM1 && currentRPM1>(TARGET_RPM - ErrorRange)) {
                volocitycheck1 = true;
            }
            else {
                volocitycheck1 = false;
            }
            if((TARGET_RPM + ErrorRange)>currentRPM2 && currentRPM2>(TARGET_RPM - ErrorRange)) {
                volocitycheck2 = true;
            }
            else {
                volocitycheck2 = false;
            }
            if(volocitycheck1||volocitycheck2){
                volocitycheck = true;
            }
            else{
                volocitycheck = false;
            }


            boolean currentYState = gamepad1.y;
            if (currentYState && !lastYState) {
                state = true;
            }

            if (state && !volocitycheck) {

                intake.setPower(0);
                blender.setPower(0);
            }
            else if (state && volocitycheck) {

                intake.setPower(1);
                blender.setPower(0.55);
            }
            lastYState = currentYState;



            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            //telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Status", "Running");
            telemetry.addData("目标 RPM", TARGET_RPM);
            telemetry.addData("当前 S1 RPM", "%.2f", currentRPM1);
            telemetry.addData("当前 S2 RPM", "%.2f", currentRPM2);
            telemetry.addData("当前航向", "%.1f°", getHeading());
            telemetry.addData("目标航向", "%.1f°", targetHeading);
            telemetry.addData("自动转向状态", isTurningToTarget ? "进行中" : "关闭");
            telemetry.update();

        }
    }

    /**
     * 执行自动转向到目标角度
     */
    private void performAutoTurn() {
        double currentHeading = getHeading();
        double headingError = currentHeading - targetHeading;

        // 如果误差在阈值内，停止转向
        if (Math.abs(headingError) <= HEADING_THRESHOLD) {
            stopMotors();
            isTurningToTarget = false;
            telemetry.addData("自动转向", "完成");
            return;
        }

        // 计算转向功率（PID控制）
        double turnPower = calculatePIDOutput(headingError);

        // 执行转向（原地旋转）
        setMecanumPower(0, 0, turnPower);
    }

    /**
     * 计算PID控制输出
     * @param error 当前误差
     * @return PID控制输出（转向功率）
     */
    private double calculatePIDOutput(double error) {
        long currentTime = System.nanoTime();
        double deltaTime = (currentTime - previousTime) / 1e9; // 转换为秒

        // 防止除零错误
        if (deltaTime == 0) {
            deltaTime = 0.01; // 默认10ms
        }

        // 比例项
        double proportional = P_TURN_GAIN * error;

        // 积分项（带积分限幅防止积分饱和）
        integralSum += error * deltaTime;
        // 积分限幅
        double maxIntegral = 0.5 / I_GAIN; // 限制积分项的最大影响
        integralSum = Range.clip(integralSum, -maxIntegral, maxIntegral);
        double integral = I_GAIN * integralSum;

        // 微分项
        double derivative = D_GAIN * (error - previousError) / deltaTime;

        // 计算总输出
        double output = proportional + integral + derivative;

        // 限制输出范围
        output = Range.clip(output, -TURN_POWER, TURN_POWER);

        // 更新状态变量
        previousError = error;
        previousTime = currentTime;

        return output;
    }

    /**
     * 重置PID控制器状态
     */
    private void resetPID() {
        previousError = 0;
        integralSum = 0;
        previousTime = System.nanoTime();
    }

    /**
     * 设置麦克纳姆轮功率（原地旋转）
     * @param drive 前后移动功率
     * @param strafe 左右平移功率
     * @param turn 旋转功率
     */
    private void setMecanumPower(double drive, double strafe, double turn) {
        // 麦克纳姆轮功率计算
        double leftFrontPower = drive + strafe + turn;
        double rightFrontPower = drive - strafe - turn;
        double leftBackPower = drive - strafe + turn;
        double rightBackPower = drive + strafe - turn;

        // 归一化功率值，确保不超过±1.0
        double maxPower = Math.max(Math.max(
                        Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));

        if (maxPower > 1.0) {
            leftFrontPower /= maxPower;
            rightFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightBackPower /= maxPower;
        }

        // 设置电机功率
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    /**
     * 停止所有驱动电机
     */
    private void stopMotors() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    /**
     * 计算航向误差（考虑角度环绕）
     * @param currentHeading 当前航向
     * @param targetHeading 目标航向
     * @return 标准化后的误差（-180 到 180度）
     */
    private double calculateHeadingError(double currentHeading, double targetHeading) {
        double error = targetHeading - currentHeading;

        // 标准化误差到 -180 到 180 度范围
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        return error;
    }

    /**
     * 获取当前机器人航向（偏航角）
     * @return 当前航向角度（度）
     */
    private double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
