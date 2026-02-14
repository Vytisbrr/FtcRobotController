package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class REVStarterBotTeleOpAutoJava extends LinearOpMode {

  private DcMotor flywheel;
  private DcMotor coreHex;
  private DcMotor frontLeft;
  private DcMotor backLeft;
  private DcMotor frontRight;
  private DcMotor backRight;
  private DcMotor intake;
  private Servo hood;
  private int detectionStableCount = 0;
  private boolean lastDetectionState = false;
  private double currentHoodSetpoint = 0;
  private static final int bankVelocity = 1400;
  private static final int farVelocity = 1700;
  private static final int maxVelocity = 1800;
  private static final int shootVelocity = 1050;
  int targetID = 20;
  private static final String TELEOP = "TELEOP";
  private static final String AUTO_BLUE_GOAL = "AUTO BLUE GOAL";
  private static final String AUTO_BLUE_BACK = "AUTO BLUE BACK";
  private static final String AUTO_RED_BACK = "AUTO RED BACK";
  private static final String AUTO_RED_GOAL = "AUTO RED GOAL";
  private static final String AUTO_FORWARD = "AUTO FORWARD";
  private String operationSelected = TELEOP;
  private double WHEELS_INCHES_TO_TICKS = (28 * 5 * 3) / (3 * Math.PI);

  private double hoodOffset = 0.72;
  private ElapsedTime autoLaunchTimer = new ElapsedTime();
  private ElapsedTime autoDriveTimer = new ElapsedTime();
  AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
  private double[][] shooterLUT = {
          {24.0, 1330},
          {48.0, 1430},
          {72.0, 1550},
          {96.0, 1600},
          {120.0, 1700},
          {150.0, 1800},
          {180.0, 1900},
          {210.0, 2000}
  };

  @Override
  public void runOpMode() {
    aprilTagWebcam.init(hardwareMap, telemetry);
    flywheel = hardwareMap.get(DcMotor.class, "flywheel");
    coreHex = hardwareMap.get(DcMotor.class, "coreHex");
    frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
    backLeft = hardwareMap.get(DcMotor.class, "backLeft");
    frontRight = hardwareMap.get(DcMotor.class, "frontRight");
    backRight = hardwareMap.get(DcMotor.class, "backRight");
    intake = hardwareMap.get(DcMotor.class, "intake");
    hood = hardwareMap.get(Servo.class, "hood");
    // Establishing the direction and mode for the motors
    flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    flywheel.setDirection(DcMotor.Direction.REVERSE);
    coreHex.setDirection(DcMotor.Direction.REVERSE);
    frontLeft.setDirection(DcMotor.Direction.REVERSE);
    backLeft.setDirection(DcMotor.Direction.REVERSE);
    intake.setDirection(DcMotor.Direction.REVERSE);
    hood.setPosition(hoodOffset);

    //On initilization the Driver Station will prompt for which OpMode should be run - Auto Blue, Auto Red, or TeleOp
    while (opModeInInit()) {
      operationSelected = selectOperation(operationSelected, gamepad1.psWasPressed());
      telemetry.update();
    }
    waitForStart();
    if (operationSelected.equals(AUTO_BLUE_GOAL)) {
      doAutoBlueGoal();
    } else if (operationSelected.equals(AUTO_BLUE_BACK)) {
      doAutoBlueBack();
    } else if (operationSelected.equals(AUTO_RED_GOAL)) {
      doAutoRedGoal();
    } else if (operationSelected.equals(AUTO_RED_BACK)) {
      doAutoRedBack();
    } else if (operationSelected.equals(AUTO_FORWARD)) {
      doAutoForward();
    } else {
      doTeleOp();

    }
  }

  /**
   * If the PS/Home button is pressed, the robot will cycle through the OpMode options following the if/else statement here.
   * The telemetry readout to the Driver Station App will update to reflect which is currently selected for when "play" is pressed.
   */
  private String selectOperation(String state, boolean cycleNext) {
    if (cycleNext) {
      if (state.equals(TELEOP)) {
        state = AUTO_BLUE_GOAL;
      } else if (state.equals(AUTO_BLUE_GOAL)) {
        state = AUTO_BLUE_BACK;
      } else if (state.equals(AUTO_BLUE_BACK)) {
        state = AUTO_RED_GOAL;
      } else if (state.equals(AUTO_RED_GOAL)) {
        state = AUTO_RED_BACK;
      }else if (state.equals(AUTO_RED_BACK)) {
        state = AUTO_FORWARD;
      }else if (state.equals(AUTO_FORWARD)) {
        state = TELEOP;
      } else {
        telemetry.addData("WARNING", "Unknown Operation State Reached - Restart Program");
      }
    }
    telemetry.addLine("Press Home Button to cycle options");
    telemetry.addData("CURRENT SELECTION", state);
    if (state.equals(AUTO_BLUE_GOAL) || state.equals(AUTO_RED_GOAL) || state.equals(AUTO_BLUE_BACK) || state.equals(AUTO_RED_BACK) || state.equals(AUTO_FORWARD)) {
      telemetry.addLine("Please remember to enable the AUTO timer!");
    }
    telemetry.addLine("Press START to start your program");
    return state;
  }

  //TeleOp Code

  /**
   * If TeleOp was selected or defaulted to, the following will be active upon pressing "play".
   */
  private void doTeleOp() {
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        splitStickArcadeDrive();
        setFlywheelVelocity(); // This now runs the flywheel constantly
        manualCoreHexControl();
        double targetPosition = hoodOffset + getHoodSetpoint();
        hood.setPosition(Math.min(targetPosition, 0.85));
        aprilTagWebcam.update();
        telemetry.addData("Hood angle", hood.getPosition());
        telemetry.addData("Flywheel Target", ((DcMotorEx) flywheel).getVelocity());
        telemetry.update();

        // REMOVED: ((DcMotorEx) flywheel).setVelocity(shootVelocity);
        // (This line was previously overriding all your logic)
      }
    }
  }

  private double getHoodSetpoint(){
    org.firstinspires.ftc.vision.apriltag.AprilTagDetection detection = aprilTagWebcam.getTagBySpecificId(targetID);
    boolean tagDetected = (detection != null && detection.ftcPose != null);
    
    // Only update if detection state is stable for 3 frames
    if (tagDetected == lastDetectionState) {
      detectionStableCount++;
    } else {
      detectionStableCount = 0;
      lastDetectionState = tagDetected;
    }
    
    if (detectionStableCount >= 3) {
      if (tagDetected) {
        currentHoodSetpoint = detection.ftcPose.range * 0.001;
        telemetry.addData("dist", detection.ftcPose.range);
      } else {
        currentHoodSetpoint = 0;
      }
    }
    
    return currentHoodSetpoint;
  }

  private void setFlywheelVelocity() {
    // 1. Get distance from AprilTag
    org.firstinspires.ftc.vision.apriltag.AprilTagDetection detection = aprilTagWebcam.getTagBySpecificId(targetID);

    double currentRange = -1;
    if (detection != null && detection.ftcPose != null) {
      currentRange = detection.ftcPose.range;
    }

    // 2. Determine target velocity (Adjustable or Fallback)
    double targetVelocity = 0;

    if (currentRange != -1) {
      // Interpolate distance from shooterLUT
      if (currentRange <= shooterLUT[0][0]) {
        targetVelocity = shooterLUT[0][1];
      } else if (currentRange >= shooterLUT[shooterLUT.length - 1][0]) {
        targetVelocity = shooterLUT[shooterLUT.length - 1][1];
      } else {
        for (int i = 0; i < shooterLUT.length - 1; i++) {
          if (currentRange >= shooterLUT[i][0] && currentRange <= shooterLUT[i+1][0]) {
            double d1 = shooterLUT[i][0], d2 = shooterLUT[i+1][0];
            double v1 = shooterLUT[i][1], v2 = shooterLUT[i+1][1];
            targetVelocity = v1 + (v2 - v1) * ((currentRange - d1) / (d2 - d1));
            break;
          }
        }
      }
    } else {
      // Use bankVelocity if no tag is seen
      targetVelocity = bankVelocity;
    }

    // 3. APPLY VELOCITY CONSTANTLY
    ((DcMotorEx) flywheel).setVelocity(targetVelocity);

    // 4. CORE HEX ONLY ON BUMPERS
    if (gamepad1.left_bumper || gamepad1.right_bumper) {
      coreHex.setPower(1.0);
    } else {
      coreHex.setPower(0);
    }

    // Diagnostics
    telemetry.addData("Shooter Mode", currentRange == -1 ? "FALLBACK (Bank)" : "AUTO-ADJUSTING");
    telemetry.addData("Target RPM", targetVelocity);
  }



  private void splitStickArcadeDrive() {
    // y = forward/backward, x = strafe left/right, rx = turn left/right
    double y = -gamepad1.left_stick_x;
    double x = gamepad1.left_stick_y * 1.1;
    double rx = -gamepad1.right_stick_x;

    // Check if the button is currently being HELD
    if (gamepad1.circle) {

      // Get the specific tag from webcam helper class
      org.firstinspires.ftc.vision.apriltag.AprilTagDetection detection = aprilTagWebcam.getTagBySpecificId(targetID);

      if (detection != null && detection.ftcPose != null) {
        double TURN_GAIN = 0.03;

        // Override the manual rx with the bearing (the angle to the tag)
        rx = detection.ftcPose.bearing * TURN_GAIN;

        telemetry.addData("Auto-Face", "Targeting ID", targetID);
        telemetry.addData("Bearing", detection.ftcPose.bearing);
      } else {
        // If button is held but tag is not in view, stop turning
        rx = 0;
        telemetry.addData("Auto-Face", "ID NOT IN VIEW", targetID);
      }
    }

    // Denominator ensures all powers maintain the same ratio
    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

    double frontLeftPower = (y + x + rx) / denominator;
    double backLeftPower = (y - x + rx) / denominator;
    double frontRightPower = (y - x - rx) / denominator;
    double backRightPower = (y + x - rx) / denominator;

    frontLeft.setPower(frontLeftPower);
    backLeft.setPower(backLeftPower);
    frontRight.setPower(frontRightPower);
    backRight.setPower(backRightPower);
  }



  /**
   * Manual control for the Core Hex powered feeder
   */
  private void manualCoreHexControl() {
    // Manual control for the intake
    if (gamepad1.cross) {
      intake.setPower(-1.0);
    } else if (gamepad1.triangle) {
      intake.setPower(0.0);
    }
    if (gamepad1.dpad_left) {
      targetID = 20;
    } else if (gamepad1.dpad_right) {
      targetID = 24;
    }
  }

  /**
   * This if/else statement contains the controls for the flywheel, both manual and auto.
   * Circle and Square will spin up ONLY the flywheel to the target velocity set.
   * The bumpers will activate the flywheel, and Core Hex feeder
   */



//Autonomous Code
//For autonomous, the robot will launch the pre-loaded 3 balls then back away from the goal, turn, and back up off the launch line.

  /**
   * For autonomous, the robot is using a timer and encoders on the drivetrain to move away from the target.
   * This method contains the math to be used with the inputted distance for the encoders, resets the elapsed timer, and
   * provides a check for it to run so long as the motors are busy and the timer has not run out.
   */

  private void doAutoBlueGoal() {
    if (opModeIsActive()) {
      telemetry.addData("RUNNING OPMODE", operationSelected);
      telemetry.update();
      // Fire balls
      autoLaunchTimer.reset();
      while (opModeIsActive() && autoLaunchTimer.milliseconds() < 10000) {

        telemetry.addData("Launcher Countdown", autoLaunchTimer.seconds());
        telemetry.update();
      }
      ((DcMotorEx) flywheel).setVelocity(0);
      coreHex.setPower(0.0);
      backLeft.setPower(-1.0);
      frontLeft.setPower(-1.0);
      backRight.setPower(-1.0);
      frontRight.setPower(-1.0);
      sleep(500);
      backLeft.setPower(0.5);
      frontLeft.setPower(0.5);
      backRight.setPower(-0.5);
      frontRight.setPower(-0.5);
      sleep(500);
      backLeft.setPower(0.75);
      frontLeft.setPower(0.75);
      backRight.setPower(0.75);
      frontRight.setPower(0.75);
      sleep(500);
      backLeft.setPower(0.0);
      frontLeft.setPower(0.0);
      backRight.setPower(0.0);
      frontRight.setPower(0.0);
    }
  }

  private void doAutoBlueBack() {
    if (opModeIsActive()) {
      telemetry.addData("RUNNING OPMODE", operationSelected);
      telemetry.update();
      backLeft.setPower(1.0);
      frontLeft.setPower(1.0);
      backRight.setPower(1.0);
      frontRight.setPower(1.0);
      sleep(1125);
      backLeft.setPower(0.0);
      frontLeft.setPower(0.0);
      backRight.setPower(0.0);
      frontRight.setPower(0.0);
      sleep(100);
      backLeft.setPower(0.5);
      frontLeft.setPower(0.5);
      backRight.setPower(-0.5);
      frontRight.setPower(-0.5);
      sleep(500);
      backLeft.setPower(0.0);
      frontLeft.setPower(0.0);
      backRight.setPower(0.0);
      frontRight.setPower(0.0);
      sleep(100);
      backLeft.setPower(1.0);
      frontLeft.setPower(1.0);
      backRight.setPower(1.0);
      frontRight.setPower(1.0);
      sleep(750);
      // Fire balls
      autoLaunchTimer.reset();
      while (opModeIsActive() && autoLaunchTimer.milliseconds() < 10000) {

        telemetry.addData("Launcher Countdown", autoLaunchTimer.seconds());
        telemetry.update();
      }
      ((DcMotorEx) flywheel).setVelocity(0);
    }
  }

  /**
   * Red Alliance Autonomous
   * The robot will fire the pre-loaded balls until the 10 second timer ends.
   * Then it will back away from the goal and off the launch line.
   */
  private void doAutoRedGoal() {
    if (opModeIsActive()) {
      telemetry.addData("RUNNING OPMODE", operationSelected);
      telemetry.update();
      // Fire balls
      autoLaunchTimer.reset();
      while (opModeIsActive() && autoLaunchTimer.milliseconds() < 10000) {

        telemetry.addData("Launcher Countdown", autoLaunchTimer.seconds());
        telemetry.update();
      }
      coreHex.setPower(0);
      backLeft.setPower(-1);
      frontLeft.setPower(-1);
      backRight.setPower(-1);
      frontRight.setPower(-1);
      sleep(500);
      frontLeft.setPower(0.5);
      backLeft.setPower(0.5);
      frontRight.setPower(-0.5);
      backRight.setPower(-0.5);
      sleep(500);
      backLeft.setPower(0);
      frontLeft.setPower(0);
      backRight.setPower(0);
      frontRight.setPower(0);
      sleep(100);
      frontLeft.setPower(0.75);
      backLeft.setPower(0.75);
      frontRight.setPower(0.75);
      backRight.setPower(0.75);
      sleep(500);
      backLeft.setPower(0);
      frontLeft.setPower(0);
      backRight.setPower(0);
      frontRight.setPower(0);
    }
  }

  private void doAutoRedBack() {
    if (opModeIsActive()) {
      telemetry.addData("RUNNING OPMODE", operationSelected);
      telemetry.update();
      backLeft.setPower(1);
      frontLeft.setPower(1);
      backRight.setPower(1);
      frontRight.setPower(1);
      sleep(1125);
      backLeft.setPower(0);
      frontLeft.setPower(0);
      backRight.setPower(0);
      frontRight.setPower(0);
      sleep(100);
      backLeft.setPower(-0.5);
      frontLeft.setPower(-0.5);
      backRight.setPower(0.5);
      frontRight.setPower(0.5);
      sleep(500);
      backLeft.setPower(0);
      frontLeft.setPower(0);
      backRight.setPower(0);
      frontRight.setPower(0);
      sleep(100);
      backLeft.setPower(1);
      frontLeft.setPower(1);
      backRight.setPower(1);
      frontRight.setPower(1);
      sleep(750);
      // Fire balls
      autoLaunchTimer.reset();
      while (opModeIsActive() && autoLaunchTimer.milliseconds() < 10000) {

        telemetry.addData("Launcher Countdown", autoLaunchTimer.seconds());
        telemetry.update();
      }
      ((DcMotorEx) flywheel).setVelocity(0);
    }
  }

  private void doAutoForward() {
    if (opModeIsActive()) {
      telemetry.addData("RUNNING OPMODE", operationSelected);
      telemetry.update();
      backLeft.setPower(1);
      frontLeft.setPower(1);
      backRight.setPower(1);
      frontRight.setPower(1);
      sleep(500);
      backLeft.setPower(0);
      frontLeft.setPower(0);
      backRight.setPower(0);
      frontRight.setPower(0);
      sleep(0);
    }
  }
}
