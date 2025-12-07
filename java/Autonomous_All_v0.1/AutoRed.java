package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "AutoRed (Blocks to Java)")
public class AutoRed extends LinearOpMode {

  private DcMotor lb;
  private DcMotor rf;
  private DcMotor s1;
  private DcMotor s2;
  private DcMotor load;
  private DcMotor lf;
  private DcMotor rb;
  private DcMotor intake;

  /**
   * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
   * Comment Blocks show where to place Initialization code (runs once, after touching the
   * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
   * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
   * Stopped).
   */
  @Override
  public void runOpMode() {
    int set;

    lb = hardwareMap.get(DcMotor.class, "lb");
    rf = hardwareMap.get(DcMotor.class, "rf");
    s1 = hardwareMap.get(DcMotor.class, "s1");
    s2 = hardwareMap.get(DcMotor.class, "s2");
    load = hardwareMap.get(DcMotor.class, "load");
    lf = hardwareMap.get(DcMotor.class, "lf");
    rb = hardwareMap.get(DcMotor.class, "rb");
    intake = hardwareMap.get(DcMotor.class, "intake");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      lb.setDirection(DcMotor.Direction.REVERSE);
      rf.setDirection(DcMotor.Direction.REVERSE);
      s1.setDirection(DcMotor.Direction.REVERSE);
      s2.setDirection(DcMotor.Direction.REVERSE);
      load.setDirection(DcMotor.Direction.REVERSE);
      lb.setPower(1);
      lf.setPower(1);
      rb.setPower(1);
      rf.setPower(1);
      sleep(800);
      lb.setPower(0.5);
      lf.setPower(0.5);
      rb.setPower(-0.5);
      rf.setPower(-0.5);
      sleep(80);
      lf.setPower(1);
      lb.setPower(1);
      rb.setPower(1);
      rf.setPower(1);
      lb.setPower(0);
      lf.setPower(0);
      rb.setPower(0);
      rf.setPower(0);
      s1.setPower(0.35);
      s2.setPower(0.35);
      sleep(1500);
      load.setPower(1);
      while (opModeIsActive()) {
        // Put loop blocks here.
        telemetry.update();
        intake.setPower(1);
        sleep(1000);
        intake.setPower(-1);
        sleep(100);
      }
      load.setPower(0);
      s1.setPower(0);
      s2.setPower(0);
      intake.setPower(0);
    }
  }
}
