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
  double servoPosition;
  private static final int bankVelocity = 1200;
  private static final int farVelocity = 1700;
  private static final int maxVelocity = 2200;
  private static final int shootVelocity = 1050;
  private static int lockoncount = 0;
  private static int lockoncountmax = 200;
  private static int override = 0;
  private static double lastflywheelspeed = 0;
  private static double lasthooddistance = 0;
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
          {24.0, 1200},
          {48.0, 1330},
          {72.0, 1400},
          {96.0, 1500},
          {120.0, 1570},
          {150.0, 1630},
          {180.0, 1690}
  };
  private static int overridebank = 0;

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
      } else if (state.equals(AUTO_FORWARD)) {
        state = TELEOP;
      } else if (state.equals(AUTO_RED_BACK)) {
        state = AUTO_FORWARD;
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
        if (override == 0) {
          hood.setPosition(Math.min(hoodOffset + getHoodSetpoint(), 0.867));
        }
        aprilTagWebcam.update();
        if (targetID==20){
          telemetry.addData("Targeting Blue, ID:", targetID);
        } else if (targetID==24) {
          telemetry.addData("Targeting Red, ID:", targetID);
        }
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
    double Distance = -1;
    if (lockoncount == 0 && overridebank == 0) {
      // Locked on
      Distance = detection.ftcPose.range;
      lasthooddistance = Distance;
      telemetry.addData("dist", Distance);
      return Distance * 0.0012;
    } else if (lockoncount > 0 && lockoncount < lockoncountmax && overridebank == 0) {
      // Lock on Buffer
      Distance = lasthooddistance;
      telemetry.addData("dist", Distance);
      return Distance * 0.0012;
    } else if (lockoncount >= lockoncountmax && overridebank == 0) {
      // Lock on lost fallback
      telemetry.addData("dist", 0);
      return 0;
    } if (overridebank == 1) {
      return 0;
    }
    telemetry.addData("NEVER SEE THIS", 0);
    return 0;
  }

  private void setFlywheelVelocity() {
    // 1. Get distance from AprilTag
    org.firstinspires.ftc.vision.apriltag.AprilTagDetection detection = aprilTagWebcam.getTagBySpecificId(targetID);

    double currentRange = -1;
    if (detection != null && detection.ftcPose != null) {
      currentRange = detection.ftcPose.range;
    }

    // 2. Determine target velocity (Adjustable or Fallback)
    double targetVelocity = lastflywheelspeed;
    // Locked on code
    if (currentRange != -1 && override == 0) {
      lockoncount = 0;
      telemetry.addData("Locked", lockoncount);
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
    // Lock on Lost
    } else if (override == 0 && currentRange == -1 && lockoncount >= lockoncountmax) {
      // Use bankVelocity if no tag is seen
      targetVelocity = bankVelocity;
      telemetry.addData("Locked ", "Lost");
    } else if (override == 1) {
      targetVelocity = 1900;
    }
    if (currentRange == -1 && lockoncount < lockoncountmax && overridebank == 0) {
      lockoncount += 1;
      ((DcMotorEx) flywheel).setVelocity(lastflywheelspeed);
      telemetry.addData("Locked", lockoncount);

    } else if (currentRange == -1 && lockoncount >= lockoncountmax) {
      lockoncount = lockoncountmax;
    } else if (overridebank == 1) {
      ((DcMotorEx) flywheel).setVelocity(bankVelocity);
    }
    // 3. APPLY VELOCITY CONSTANTLY
    if (lockoncount == 0 || lockoncount >= lockoncountmax && overridebank == 0) {
      ((DcMotorEx) flywheel).setVelocity(targetVelocity);
    }
    // 4. CORE HEX ONLY ON BUMPERS
    if (gamepad1.right_trigger >= 0.25) {
      sleep(50);
      if (((DcMotorEx) flywheel).getVelocity() >= targetVelocity - 80 && ((DcMotorEx) flywheel).getVelocity() <= targetVelocity + 80) {
        coreHex.setPower(0.8);
      } else {
        coreHex.setPower(0);
      }
    }
    else {
      coreHex.setPower(0);
    }
    // Diagnostics
    telemetry.addData("Shooter Mode", currentRange == -1 && lockoncount == lockoncountmax ? "FALLBACK (Bank)" : "AUTO-ADJUSTING");
    telemetry.addData("Target RPM", targetVelocity);
    telemetry.addData("lastvelocity", lastflywheelspeed);
    if (currentRange != -1) {
      lastflywheelspeed = targetVelocity;
    }
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
    if (gamepad1.left_trigger >= 0.25) {
      intake.setPower(-1.0);
    } else {
      intake.setPower(0.0);
    }
    if (gamepad1.left_bumper) {
      intake.setPower(1.0);
    }
    if (gamepad1.dpad_left) {
      targetID = 20;
    } else if (gamepad1.dpad_right) {
      targetID = 24;
    }
    if (gamepad1.dpad_up) {
      override = 1;
      hood.setPosition(0.867);
    } else {
      override = 0;
    }
    if (gamepad1.square) {
      overridebank = 1;
    }
    else {
      overridebank = 0;
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
      setFlywheelVelocity();
      hood.setPosition(hoodOffset);
      sleep(500);
      intake.setPower(-1);
      coreHex.setPower(1);
      sleep(3000);
      intake.setPower(0);
      coreHex.setPower(0);
      moveBack(0.5, 1);
      turnLeft(0.45, 0.5);
      strafeLeft(1.1, 0.5);
      intake.setPower(-1);
      moveForward(1, 1);
      moveBack(1, 1);
      strafeRight(1.1, 0.5);
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
  private void strafeLeft(double seconds, double speed) {
    frontLeft.setPower(speed);
    backLeft.setPower(speed);
    frontRight.setPower(speed);
    backRight.setPower(speed);
    sleep((long)(seconds * 1000));
    frontLeft.setPower(0);
    backLeft.setPower(0);
    frontRight.setPower(0);
    backRight.setPower(0);
  }
  private void strafeRight(double seconds, double speed) {
    frontLeft.setPower(-speed);
    backLeft.setPower(-speed);
    frontRight.setPower(-speed);
    backRight.setPower(-speed);
    sleep((long)(seconds * 1000));
    frontLeft.setPower(0);
    backLeft.setPower(0);
    frontRight.setPower(0);
    backRight.setPower(0);
  }
  private void turnRight(double seconds, double speed) {
    frontLeft.setPower(-speed);
    backLeft.setPower(-speed);
    frontRight.setPower(speed);
    backRight.setPower(speed);
    sleep((long)(seconds * 1000));
    frontLeft.setPower(0);
    backLeft.setPower(0);
    frontRight.setPower(0);
    backRight.setPower(0);
  }
  private void turnLeft(double seconds, double speed) {
    frontLeft.setPower(speed);
    backLeft.setPower(speed);
    frontRight.setPower(-speed);
    backRight.setPower(-speed);
    sleep((long)(seconds * 1000));
    frontLeft.setPower(0);
    backLeft.setPower(0);
    frontRight.setPower(0);
    backRight.setPower(0);
  }
  private void moveForward(double seconds, double speed) {
    frontLeft.setPower(-speed);
    backLeft.setPower(speed);
    frontRight.setPower(speed);
    backRight.setPower(-speed);
    sleep((long)(seconds * 1000));
    frontLeft.setPower(0);
    backLeft.setPower(0);
    frontRight.setPower(0);
    backRight.setPower(0);
  }
  private void moveBack(double seconds, double speed) {
    frontLeft.setPower(speed);
    backLeft.setPower(-speed);
    frontRight.setPower(-speed);
    backRight.setPower(speed);
    sleep((long)(seconds * 1000));
    frontLeft.setPower(0);
    backLeft.setPower(0);
    frontRight.setPower(0);
    backRight.setPower(0);
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
      // Shooting
      ((DcMotorEx) flywheel).setVelocity(1250);
      hood.setPosition(hoodOffset);
      sleep(500);
      intake.setPower(-1);
      coreHex.setPower(1);
      sleep(3000);
      intake.setPower(0);
      coreHex.setPower(0);
      // First Station
      moveBack(0.3, 0.5);
      moveBack(0.5, 1);
      turnRight(0.45, 0.5);
      strafeRight(1, 0.5);
      intake.setPower(-1);
      sleep(1000);
      moveForward(1, 0.8);
      sleep(2000);
      moveBack(1, 0.8);
      strafeLeft(1, 0.5);
      turnLeft(0.5, 0.5);
      moveForward(0.8, 1);
      sleep(1000);
      intake.setPower(-1);
      coreHex.setPower(1);
      sleep(2000);
      // Second Station
      moveBack(0.3,0.5);
      moveBack(0.5, 1);
      turnRight(0.45, 0.5);
      strafeRight(2.2, 0.5);
      intake.setPower(-1);
      sleep(500);
      moveForward(1, 1);
      moveBack(1, 1);
      strafeLeft(2.2, 0.5);
      turnLeft(0.5, 0.5);
      moveForward(0.8, 1);
      sleep(1000);
      intake.setPower(-1);
      coreHex.setPower(1);
      sleep(2000);
      moveForward(0, 0);
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
      frontLeft.setPower(-1);
      backRight.setPower(-1);
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
