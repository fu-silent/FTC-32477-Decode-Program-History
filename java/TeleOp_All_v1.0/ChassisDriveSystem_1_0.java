package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * 底盘驱动系统管理类
 * 负责麦克纳姆轮的运动学计算和电机控制
 * 职责：
 * - 初始化四个底盘电机
 * - 实现麦克纳姆轮运动学公式
 * - 应用死区、非线性映射、功率归一化
 * - 控制四个电机的功率输出
 */
public class ChassisDriveSystem_1_0 {

    // ==================== 硬件对象 ====================
    private DcMotor motorsChassisFL;   // 前左
    private DcMotor motorsChassisRF;   // 前右
    private DcMotor motorsChassisRL;   // 后左
    private DcMotor motorsChassisRR;   // 后右

    // ==================== 状态变量 ====================
    private boolean isInitialized = false;

    // ==================== 构造函数 ====================
    public ChassisDriveSystem_1_0() {
    }

    // ==================== 初始化方法 ====================

    /**
     * 从HardwareMap初始化底盘电机
     * @param hardwareMap FTC硬件映射对象
     * @return 初始化是否成功
     */
    public boolean initialize(HardwareMap hardwareMap) {
        try {
            // 获取电机引用
            motorsChassisFL = hardwareMap.get(DcMotor.class, RobotConstants_1_0.CHASSIS_MOTOR_FRONT_LEFT_NAME);
            motorsChassisRF = hardwareMap.get(DcMotor.class, RobotConstants_1_0.CHASSIS_MOTOR_FRONT_RIGHT_NAME);
            motorsChassisRL = hardwareMap.get(DcMotor.class, RobotConstants_1_0.CHASSIS_MOTOR_BACK_LEFT_NAME);
            motorsChassisRR = hardwareMap.get(DcMotor.class, RobotConstants_1_0.CHASSIS_MOTOR_BACK_RIGHT_NAME);

            // 设置电机方向
            motorsChassisFL.setDirection(DcMotor.Direction.FORWARD);
            motorsChassisRF.setDirection(DcMotor.Direction.REVERSE);
            motorsChassisRL.setDirection(DcMotor.Direction.REVERSE);
            motorsChassisRR.setDirection(DcMotor.Direction.FORWARD);

            // 设置运行模式为无编码器模式
            motorsChassisFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorsChassisRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorsChassisRL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorsChassisRR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // 设置零功率行为为制动，确保摇杆回中时车辆快速刹停
            motorsChassisFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorsChassisRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorsChassisRL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorsChassisRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // 停止所有电机
            stop();

            isInitialized = true;
            return true;

        } catch (Exception e) {
            isInitialized = false;
            return false;
        }
    }

    /**
     * 检查是否初始化成功
     */
    public boolean isInitialized() {
        return isInitialized;
    }

    // ==================== 麦克纳姆轮运动学计算 ====================

    /**
     * 计算麦克纳姆轮的四轮功率
     * 基于摇杆输入（前后、左右、旋转）计算各轮应该输出的功率
     * 
     * 运动学公式：
     * FL = drive + strafe + turn    (前左)
     * FR = drive - strafe - turn    (前右)
     * BL = drive - strafe + turn    (后左)
     * BR = drive + strafe - turn    (后右)
     * 
     * @param inputDriveFB 前后移动输入 (-1.0 到 1.0)
     * @param inputStrafeLR 左右平移输入 (-1.0 到 1.0)
     * @param inputRotateCW 旋转输入 (-1.0 到 1.0)
     * @param useNonlinearMapping 是否使用非线性映射
     * @return 四轮功率数组 [FL, FR, BL, BR]
     */
    public double[] calculateWheelPowers(double inputDriveFB, double inputStrafeLR, double inputRotateCW, 
                                         boolean useNonlinearMapping) {
        
        // 应用死区
        inputDriveFB = applyDeadzone(inputDriveFB);
        inputStrafeLR = applyDeadzone(inputStrafeLR);
        inputRotateCW = applyDeadzone(inputRotateCW);

        // 如果所有输入都为0，则立即返回四个0功率，保证车辆刹停
        if (inputDriveFB == 0.0 && inputStrafeLR == 0.0 && inputRotateCW == 0.0) {
            return new double[]{0.0, 0.0, 0.0, 0.0};
        }

        // 应用非线性映射（仅对左摇杆，不对右摇杆）
        if (useNonlinearMapping) {
            inputDriveFB = applyNonlinearMapping(inputDriveFB);
            inputStrafeLR = applyNonlinearMapping(inputStrafeLR);
        }

        // 计算原始功率（麦克纳姆轮运动学）
        double powerFL = inputDriveFB + inputStrafeLR + inputRotateCW;
        double powerFR = inputDriveFB - inputStrafeLR - inputRotateCW;
        double powerBL = inputDriveFB + inputStrafeLR - inputRotateCW;
        double powerBR = inputDriveFB - inputStrafeLR + inputRotateCW;

        // 归一化功率（防止超过1.0）
        double maxPower = Math.max(
            Math.max(Math.abs(powerFL), Math.abs(powerFR)),
            Math.max(Math.abs(powerBL), Math.abs(powerBR))
        );

        if (maxPower > 1.0) {
            powerFL /= maxPower;
            powerFR /= maxPower;
            powerBL /= maxPower;
            powerBR /= maxPower;
        }

        // 应用最大功率限制
        powerFL *= RobotConstants_1_0.CHASSIS_MAX_MOTOR_POWER;
        powerFR *= RobotConstants_1_0.CHASSIS_MAX_MOTOR_POWER;
        powerBL *= RobotConstants_1_0.CHASSIS_MAX_MOTOR_POWER;
        powerBR *= RobotConstants_1_0.CHASSIS_MAX_MOTOR_POWER;

        return new double[]{powerFL, powerFR, powerBL, powerBR};
    }

    // ==================== 电机控制方法 ====================

    /**
     * 设置四轮电机功率
     * @param wheelPowers 四轮功率数组 [FL, FR, BL, BR]
     */
    public void setWheelPowers(double[] wheelPowers) {
        if (wheelPowers.length != 4) {
            return;
        }
        motorsChassisFL.setPower(wheelPowers[0]);
        motorsChassisRF.setPower(wheelPowers[1]);
        motorsChassisRL.setPower(wheelPowers[2]);
        motorsChassisRR.setPower(wheelPowers[3]);
    }

    /**
     * 停止所有底盘电机
     */
    public void stop() {
        motorsChassisFL.setPower(0.0);
        motorsChassisRF.setPower(0.0);
        motorsChassisRL.setPower(0.0);
        motorsChassisRR.setPower(0.0);
    }

    /**
     * 获取当前四轮功率
     * @return 四轮功率数组 [FL, FR, BL, BR]
     */
    public double[] getCurrentWheelPowers() {
        return new double[]{
            motorsChassisFL.getPower(),
            motorsChassisRF.getPower(),
            motorsChassisRL.getPower(),
            motorsChassisRR.getPower()
        };
    }

    // ==================== 辅助方法 ====================

    /**
     * 应用死区（防止摇杆漂移）
     * @param value 输入值
     * @return 应用死区后的值
     */
    private double applyDeadzone(double value) {
        if (Math.abs(value) < RobotConstants_1_0.CHASSIS_JOYSTICK_DEADZONE) {
            return 0.0;
        }
        return value;
    }

    /**
     * 应用非线性映射（平方映射）
     * 使小幅度操作更精确，大幅度保持最大值
     * @param value 输入值 (-1.0 到 1.0)
     * @return 映射后的值
     */
    private double applyNonlinearMapping(double value) {
        if (value == 0.0) {
            return 0.0;
        }
        double sign = value > 0 ? 1.0 : -1.0;
        return sign * value * value;
    }

    // ==================== 调试方法 ====================

    /**
     * 获取底盘初始化信息
     */
    public String getInitializationStatus() {
        if (!isInitialized) {
            return "底盘系统未初始化";
        }
        return "底盘系统已初始化: FL=" + motorsChassisFL.getPower() + 
               ", FR=" + motorsChassisRF.getPower() + 
               ", BL=" + motorsChassisRL.getPower() + 
               ", BR=" + motorsChassisRR.getPower();
    }
}
