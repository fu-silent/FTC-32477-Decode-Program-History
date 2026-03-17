package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class RobotHardware {
    public DcMotorEx lf, rf, lb, rb, intake, load, s1, s2;
    public IMU imu;

    // 物理常数
    public final double WHEEL_DIAMETER_MM = 101.6;
    public final double ENCODER_TICKS_PER_REV = 537.7;
    public final double TICKS_PER_MM = ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI);
    public final double SHOOTER_TICKS = 28.0;

    public void init(HardwareMap hwMap) {
        lf = hwMap.get(DcMotorEx.class, "lf");
        rf = hwMap.get(DcMotorEx.class, "rf");
        lb = hwMap.get(DcMotorEx.class, "lb");
        rb = hwMap.get(DcMotorEx.class, "rb");
        intake = hwMap.get(DcMotorEx.class, "intake");
        load = hwMap.get(DcMotorEx.class, "load");
        s1 = hwMap.get(DcMotorEx.class, "s1");
        s2 = hwMap.get(DcMotorEx.class, "s2");
        imu = hwMap.get(IMU.class, "imu");

        // 默认方向
        lf.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);
        s1.setDirection(DcMotor.Direction.FORWARD);
        s2.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        load.setDirection(DcMotor.Direction.REVERSE);

        // 设置刹车
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU初始化
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();
    }
}