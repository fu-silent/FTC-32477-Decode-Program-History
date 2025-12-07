package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

// 摄像头相关导入
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvPipeline;

// OpenCV相关导入（用于图像处理）
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Size;

/**
 * FTC32477 自动阶段控制程序
 * 适用于四个麦克纳姆轮的机器人
 * 
 * 功能：
 * - 编码器驱动的精确移动（前进、后退、平移、转向）
 * - 支持多个自动路径选择
 * - 自动执行拾取、装填、发射等动作序列
 * 
 * 注意：后两个轮子物理安装方向不同，在运动学公式中直接补偿
 * 
 * @author FTC32477
 * @version 1.0 - 自动阶段控制
 */
@Autonomous(name = "FTC32477 Auto", group = "Autonomous")
public class FTC32477_Auto extends LinearOpMode {

    // 定义四个麦克纳姆轮电机
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    
    // 拾取模块（intake）电机
    private DcMotor intake;

    // 装填模块（load）电机
    private DcMotor load;

    // 发射模块（shooter）双电机：s1、s2
    private DcMotor s1;
    private DcMotor s2;

    // 摄像头相关
    private OpenCvWebcam webcam;
    private QRCodeDetectionPipeline detectionPipeline;

    // 编码器参数（需要根据实际机器人调整）
    private static final double COUNTS_PER_MOTOR_REV = 1120; // 假设使用20:1减速箱
    private static final double DRIVE_GEAR_REDUCTION = 1.0; // 驱动齿轮减速比
    private static final double WHEEL_DIAMETER_INCHES = 4.0; // 轮子直径（英寸）
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    
    // 电机功率限制
    private static final double MAX_POWER = 0.6; // 自动阶段使用较低功率以提高精度
    private static final double TURN_POWER = 0.4; // 转向功率
    
    // 超时时间（秒）
    private static final double TIMEOUT_SECONDS = 5.0;
    
    // 计时器
    private ElapsedTime runtime = new ElapsedTime();

    // 识别结果枚举
    public enum DetectionResult {
        LEFT,      // 左侧高塔二维码
        CENTER,    // 中央指示图案二维码
        RIGHT,     // 右侧高塔二维码
        UNKNOWN    // 未识别到
    }

    @Override
    public void runOpMode() {
        // 初始化硬件
        initializeHardware();
        
        // 等待摄像头初始化完成
        sleep(1000);
        
        // 显示等待信息
        telemetry.addData("状态", "等待开始...");
        telemetry.addData("提示", "自动程序已就绪");
        telemetry.addData("摄像头", "准备识别二维码");
        telemetry.update();
        
        // 在等待开始期间可以预览识别结果
        while (!isStarted() && !isStopRequested()) {
            if (detectionPipeline != null) {
                DetectionResult previewResult = detectionPipeline.getLastResult();
                telemetry.addData("预览识别", previewResult.toString());
                telemetry.update();
            }
            sleep(100);
        }
        
        // 等待比赛开始
        waitForStart();
        
        // 执行自动路径（包含识别和执行）
        runAutonomousPath();
        
        // 停止所有电机
        stopAllMotors();
        stopAllSubsystems();
        
        telemetry.addData("状态", "自动阶段完成");
        telemetry.update();
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

            // 初始化发射模块（shooter）电机：s1、s2
            s1 = hardwareMap.get(DcMotor.class, "s1");
            s2 = hardwareMap.get(DcMotor.class, "s2");

            // 设置电机方向
            frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);
            
            // 设置拾取模块（intake）电机方向
            intake.setDirection(DcMotorSimple.Direction.FORWARD);

            // 设置装填模块（load）电机方向（反转）
            load.setDirection(DcMotorSimple.Direction.REVERSE);

            // 设置发射模块（shooter）电机s1、s2方向
            s1.setDirection(DcMotorSimple.Direction.FORWARD);
            s2.setDirection(DcMotorSimple.Direction.FORWARD);

            // 设置所有电机运行模式为使用编码器
            setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
            
            // 设置其他电机模式
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            load.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            s1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            s2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // 设置电机零功率行为
            setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            s1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            s2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // 停止所有电机
            stopAllMotors();
            stopAllSubsystems();

            // 初始化摄像头
            initializeCamera();

            telemetry.addData("状态", "硬件初始化完成");
            telemetry.update();

        } catch (Exception e) {
            telemetry.addData("错误", "硬件初始化失败: " + e.getMessage());
            telemetry.update();
        }
    }

    /**
     * 执行自动路径
     * 根据摄像头识别结果执行不同的路径
     */
    private void runAutonomousPath() {
        telemetry.addData("状态", "开始执行自动路径");
        telemetry.update();
        
        // 步骤1：识别二维码位置（在比赛开始前识别）
        telemetry.addData("状态", "正在识别二维码...");
        telemetry.update();
        
        DetectionResult detectedPosition = detectQRCode(3.0); // 最多识别3秒
        
        telemetry.addData("识别结果", detectedPosition.toString());
        telemetry.update();
        sleep(500); // 短暂暂停，确保识别结果稳定
        
        // 步骤2：根据识别结果执行不同的路径
        switch (detectedPosition) {
            case LEFT:
                executeLeftPath();
                break;
            case CENTER:
                executeCenterPath();
                break;
            case RIGHT:
                executeRightPath();
                break;
            default:
                // 如果未识别到，执行默认路径（中央路径）
                telemetry.addData("警告", "未识别到二维码，执行默认路径");
                telemetry.update();
                executeCenterPath();
                break;
        }
        
        // 关闭摄像头
        if (webcam != null) {
            webcam.stopStreaming();
        }
        
        telemetry.addData("状态", "自动路径执行完成");
        telemetry.update();
    }
    
    /**
     * 执行左侧路径（左侧高塔二维码）
     */
    private void executeLeftPath() {
        telemetry.addData("路径", "执行左侧路径");
        telemetry.update();
        
        // 示例路径：
        // 1. 前进一定距离
        // 2. 拾取物品
        // 3. 左转或右转到左侧目标位置
        // 4. 前进到目标位置
        // 5. 装填并发射
        
        // 步骤1：前进24英寸
        encoderDrive(0.5, 24, 24, 0, 5.0);
        
        // 步骤2：拾取物品（运行2秒）
        runIntake(1.0, 2.0);
        
        // 步骤3：转向到左侧目标（根据实际场地调整）
        encoderTurn(0.4, -45, 5.0); // 左转45度
        
        // 步骤4：前进到目标位置
        encoderDrive(0.5, 36, 36, 0, 5.0);
        
        // 步骤5：装填并发射
        runLoad(0.9, 1.0);
        runShooter(1.0, 2.0);
    }
    
    /**
     * 执行中央路径（中央指示图案二维码）
     */
    private void executeCenterPath() {
        telemetry.addData("路径", "执行中央路径");
        telemetry.update();
        
        // 示例路径：
        // 1. 前进一定距离
        // 2. 拾取物品
        // 3. 前进到中央目标位置
        // 4. 装填并发射
        
        // 步骤1：前进24英寸
        encoderDrive(0.5, 24, 24, 0, 5.0);
        
        // 步骤2：拾取物品（运行2秒）
        runIntake(1.0, 2.0);
        
        // 步骤3：前进到中央目标位置
        encoderDrive(0.5, 36, 36, 0, 5.0);
        
        // 步骤4：装填并发射
        runLoad(0.9, 1.0);
        runShooter(1.0, 2.0);
    }
    
    /**
     * 执行右侧路径（右侧高塔二维码）
     */
    private void executeRightPath() {
        telemetry.addData("路径", "执行右侧路径");
        telemetry.update();
        
        // 示例路径：
        // 1. 前进一定距离
        // 2. 拾取物品
        // 3. 右转到右侧目标位置
        // 4. 前进到目标位置
        // 5. 装填并发射
        
        // 步骤1：前进24英寸
        encoderDrive(0.5, 24, 24, 0, 5.0);
        
        // 步骤2：拾取物品（运行2秒）
        runIntake(1.0, 2.0);
        
        // 步骤3：转向到右侧目标（根据实际场地调整）
        encoderTurn(0.4, 45, 5.0); // 右转45度
        
        // 步骤4：前进到目标位置
        encoderDrive(0.5, 36, 36, 0, 5.0);
        
        // 步骤5：装填并发射
        runLoad(0.9, 1.0);
        runShooter(1.0, 2.0);
    }

    /**
     * 使用编码器驱动机器人移动
     * @param speed 移动速度 (0.0 到 1.0)
     * @param leftInches 左轮移动距离（英寸，正数前进，负数后退）
     * @param rightInches 右轮移动距离（英寸，正数前进，负数后退）
     * @param strafeInches 平移距离（英寸，正数右移，负数左移）
     * @param timeoutS 超时时间（秒）
     */
    private void encoderDrive(double speed, double leftInches, double rightInches, 
                             double strafeInches, double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // 计算目标编码器位置
        // 前后移动：左右轮平均
        double driveInches = (leftInches + rightInches) / 2.0;
        // 转向：左右轮差值
        double turnInches = (rightInches - leftInches) / 2.0;
        
        // 麦克纳姆轮运动学（考虑后轮安装差异）
        newFrontLeftTarget = frontLeft.getCurrentPosition() + 
                (int)((driveInches + strafeInches + turnInches) * COUNTS_PER_INCH);
        newFrontRightTarget = frontRight.getCurrentPosition() + 
                (int)((driveInches - strafeInches - turnInches) * COUNTS_PER_INCH);
        newBackLeftTarget = backLeft.getCurrentPosition() + 
                (int)(-(driveInches - strafeInches + turnInches) * COUNTS_PER_INCH);
        newBackRightTarget = backRight.getCurrentPosition() + 
                (int)(-(driveInches + strafeInches - turnInches) * COUNTS_PER_INCH);

        // 设置目标位置
        frontLeft.setTargetPosition(newFrontLeftTarget);
        frontRight.setTargetPosition(newFrontRightTarget);
        backLeft.setTargetPosition(newBackLeftTarget);
        backRight.setTargetPosition(newBackRightTarget);

        // 切换到运行到目标位置模式
        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        // 重置超时时间并开始移动
        runtime.reset();
        
        // 设置电机功率
        frontLeft.setPower(Math.abs(speed));
        frontRight.setPower(Math.abs(speed));
        backLeft.setPower(Math.abs(speed));
        backRight.setPower(Math.abs(speed));

        // 等待直到所有电机到达目标位置或超时
        while (opModeIsActive() &&
               (runtime.seconds() < timeoutS) &&
               (frontLeft.isBusy() && frontRight.isBusy() && 
                backLeft.isBusy() && backRight.isBusy())) {
            
            // 显示当前位置
            telemetry.addData("路径", "运行中...");
            telemetry.addData("目标位置", "FL:%d FR:%d BL:%d BR:%d",
                    newFrontLeftTarget, newFrontRightTarget, 
                    newBackLeftTarget, newBackRightTarget);
            telemetry.addData("当前位置", "FL:%d FR:%d BL:%d BR:%d",
                    frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(),
                    backLeft.getCurrentPosition(), backRight.getCurrentPosition());
            telemetry.update();
        }

        // 停止所有电机
        stopAllMotors();

        // 切换回使用编码器模式
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * 使用编码器转向
     * @param speed 转向速度 (0.0 到 1.0)
     * @param degrees 转向角度（度，正数右转，负数左转）
     * @param timeoutS 超时时间（秒）
     */
    private void encoderTurn(double speed, double degrees, double timeoutS) {
        // 计算转向所需的轮子移动距离
        // 假设轮距为16英寸（需要根据实际机器人调整）
        double wheelBase = 16.0; // 英寸
        double arcLength = (degrees / 360.0) * Math.PI * wheelBase;
        
        // 左转：左轮后退，右轮前进
        // 右转：左轮前进，右轮后退
        double leftInches = -arcLength;
        double rightInches = arcLength;
        
        encoderDrive(speed, leftInches, rightInches, 0, timeoutS);
    }

    /**
     * 简单前进/后退（不使用编码器，基于时间）
     * @param speed 速度 (0.0 到 1.0，正数前进，负数后退)
     * @param timeSeconds 运行时间（秒）
     */
    private void timeDrive(double speed, double timeSeconds) {
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeSeconds) {
            double[] powers = calculateMecanumDrive(speed, 0, 0);
            setMotorPowers(powers);
        }
        stopAllMotors();
    }

    /**
     * 简单平移（不使用编码器，基于时间）
     * @param speed 速度 (0.0 到 1.0，正数右移，负数左移)
     * @param timeSeconds 运行时间（秒）
     */
    private void timeStrafe(double speed, double timeSeconds) {
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeSeconds) {
            double[] powers = calculateMecanumDrive(0, speed, 0);
            setMotorPowers(powers);
        }
        stopAllMotors();
    }

    /**
     * 计算麦克纳姆轮运动学（与TeleOp版本相同）
     * @param drive 前后移动 (-1 到 1)
     * @param strafe 左右平移 (-1 到 1)
     * @param turn 原地旋转 (-1 到 1)
     * @return 四个轮子的功率数组 [FL, FR, BL, BR]
     */
    private double[] calculateMecanumDrive(double drive, double strafe, double turn) {
        // 麦克纳姆轮运动学公式（考虑后轮物理安装差异）
        double frontLeftPower = drive + strafe + turn;
        double frontRightPower = drive - strafe - turn;
        // 后轮由于物理安装方向不同，需要调整公式
        double backLeftPower = -(drive - strafe + turn);
        double backRightPower = -(drive + strafe - turn);

        // 归一化功率（确保不超过1.0）
        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                                   Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // 应用最大功率限制
        frontLeftPower *= MAX_POWER;
        frontRightPower *= MAX_POWER;
        backLeftPower *= MAX_POWER;
        backRightPower *= MAX_POWER;

        return new double[]{frontLeftPower, frontRightPower, backLeftPower, backRightPower};
    }

    /**
     * 运行拾取模块
     * @param power 功率 (0.0 到 1.0，正数正向，负数反向)
     * @param timeSeconds 运行时间（秒）
     */
    private void runIntake(double power, double timeSeconds) {
        runtime.reset();
        intake.setPower(power);
        while (opModeIsActive() && runtime.seconds() < timeSeconds) {
            telemetry.addData("拾取模块", "运行中...");
            telemetry.update();
        }
        intake.setPower(0);
    }

    /**
     * 运行装填模块
     * @param power 功率 (0.0 到 1.0，正数正向，负数反向)
     * @param timeSeconds 运行时间（秒）
     */
    private void runLoad(double power, double timeSeconds) {
        runtime.reset();
        load.setPower(power);
        while (opModeIsActive() && runtime.seconds() < timeSeconds) {
            telemetry.addData("装填模块", "运行中...");
            telemetry.update();
        }
        load.setPower(0);
    }

    /**
     * 运行发射模块
     * @param power 功率 (0.0 到 1.0)
     * @param timeSeconds 运行时间（秒）
     */
    private void runShooter(double power, double timeSeconds) {
        runtime.reset();
        double actualPower = -power; // 保持与TeleOp一致的方向
        s1.setPower(actualPower);
        s2.setPower(actualPower);
        while (opModeIsActive() && runtime.seconds() < timeSeconds) {
            telemetry.addData("发射模块", "运行中...");
            telemetry.update();
        }
        s1.setPower(0);
        s2.setPower(0);
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
     * 设置所有电机零功率行为
     */
    private void setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        frontLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
    }

    /**
     * 停止所有驱动电机
     */
    private void stopAllMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    /**
     * 停止所有子系统
     */
    private void stopAllSubsystems() {
        intake.setPower(0);
        load.setPower(0);
        s1.setPower(0);
        s2.setPower(0);
    }

    /**
     * 初始化摄像头
     */
    private void initializeCamera() {
        try {
            // 获取摄像头硬件
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            
            WebcamName webcamName = hardwareMap.get(WebcamName.class, "camera");
            
            // 创建摄像头实例
            webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
            
            // 创建识别管道
            detectionPipeline = new QRCodeDetectionPipeline();
            webcam.setPipeline(detectionPipeline);
            
            // 异步打开摄像头
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    // 设置摄像头分辨率（根据实际需要调整）
                    webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                    telemetry.addData("摄像头", "已启动");
                    telemetry.update();
                }

                @Override
                public void onError(int errorCode) {
                    telemetry.addData("摄像头错误", "错误代码: " + errorCode);
                    telemetry.update();
                }
            });
            
            telemetry.addData("摄像头", "正在初始化...");
            telemetry.update();
            
        } catch (Exception e) {
            telemetry.addData("摄像头错误", "初始化失败: " + e.getMessage());
            telemetry.update();
        }
    }

    /**
     * 识别二维码位置
     * @param timeoutSeconds 超时时间（秒）
     * @return 识别结果
     */
    private DetectionResult detectQRCode(double timeoutSeconds) {
        runtime.reset();
        DetectionResult result = DetectionResult.UNKNOWN;
        
        while (opModeIsActive() && runtime.seconds() < timeoutSeconds) {
            result = detectionPipeline.getLastResult();
            
            if (result != DetectionResult.UNKNOWN) {
                telemetry.addData("识别结果", result.toString());
                telemetry.update();
                break;
            }
            
            telemetry.addData("识别状态", "识别中...");
            telemetry.addData("已用时间", "%.2f秒", runtime.seconds());
            telemetry.update();
            
            sleep(50); // 短暂等待
        }
        
        return result;
    }

    /**
     * 二维码识别管道
     * 使用颜色区域检测方法识别二维码位置
     */
    static class QRCodeDetectionPipeline extends OpenCvPipeline {
        // 识别结果
        private volatile DetectionResult lastResult = DetectionResult.UNKNOWN;
        
        // 图像处理相关
        private Mat hsvMat = new Mat();
        private Mat mask = new Mat();
        private Mat hierarchy = new Mat();
        
        // 检测区域定义（根据摄像头视野调整）
        // 假设图像分为三个区域：左、中、右
        private Rect leftRegion = new Rect(0, 0, 213, 480);      // 左侧区域
        private Rect centerRegion = new Rect(213, 0, 214, 480); // 中央区域
        private Rect rightRegion = new Rect(427, 0, 213, 480);   // 右侧区域
        
        // 颜色阈值（HSV格式，需要根据实际二维码颜色调整）
        // 这里假设二维码是某种特定颜色，需要根据实际情况调整
        private Scalar lowerBound = new Scalar(0, 50, 50);   // 颜色下限
        private Scalar upperBound = new Scalar(180, 255, 255); // 颜色上限
        
        @Override
        public Mat processFrame(Mat input) {
            // 转换为HSV色彩空间
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
            
            // 创建颜色掩码（根据实际二维码颜色调整）
            // 这里使用一个通用的方法，实际使用时需要根据二维码的具体特征调整
            Core.inRange(hsvMat, lowerBound, upperBound, mask);
            
            // 计算三个区域的像素密度
            double leftDensity = calculateRegionDensity(mask, leftRegion);
            double centerDensity = calculateRegionDensity(mask, centerRegion);
            double rightDensity = calculateRegionDensity(mask, rightRegion);
            
            // 根据密度判断二维码位置
            // 阈值需要根据实际测试调整
            double threshold = 0.1; // 像素密度阈值
            
            if (centerDensity > threshold && centerDensity > leftDensity && centerDensity > rightDensity) {
                lastResult = DetectionResult.CENTER;
            } else if (leftDensity > threshold && leftDensity > rightDensity) {
                lastResult = DetectionResult.LEFT;
            } else if (rightDensity > threshold) {
                lastResult = DetectionResult.RIGHT;
            } else {
                lastResult = DetectionResult.UNKNOWN;
            }
            
            // 在图像上绘制检测区域（用于调试）
            Imgproc.rectangle(input, leftRegion, new Scalar(255, 0, 0), 2);
            Imgproc.rectangle(input, centerRegion, new Scalar(0, 255, 0), 2);
            Imgproc.rectangle(input, rightRegion, new Scalar(0, 0, 255), 2);
            
            // 显示识别结果
            String resultText = "Result: " + lastResult.toString();
            Imgproc.putText(input, resultText, new Point(10, 30),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255, 255, 255), 2);
            
            return input;
        }
        
        /**
         * 计算区域内的像素密度
         */
        private double calculateRegionDensity(Mat mask, Rect region) {
            Mat roi = new Mat(mask, region);
            int whitePixels = Core.countNonZero(roi);
            int totalPixels = region.width * region.height;
            return (double) whitePixels / totalPixels;
        }
        
        /**
         * 获取最后一次识别结果
         */
        public DetectionResult getLastResult() {
            return lastResult;
        }
        
        /**
         * 设置颜色阈值（用于调整识别参数）
         */
        public void setColorThreshold(Scalar lower, Scalar upper) {
            lowerBound = lower;
            upperBound = upper;
        }
    }
}

