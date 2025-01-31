package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Disabled
@Autonomous(name="Final Auto Sample", group="FTC Lib")
public class Auto_Final_Sample extends LinearOpMode
{
    // april tag vars
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 12; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    // Create an instance of the sensor
    SparkFunOTOS myOtos;

    Gamepad.RumbleEffect customRumbleEffect;    // Use to build a custom rumble sequence.

    double oldTime = 0;
    int bucketVerticalPosition = 0;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor vMotor;
    // input motors exactly as shown below
    MecanumDrive mecanum;
    GamepadEx gpA, gpB;

    long attackStartTime =  0;
    long liftStartTime = 0;
    long odoResetTime = 0;

    Servo bucketServo, ledServo, armServo, pincherServo,linkServo;
    double pincherPosition = STEMperFiConstants.PINCHER_CLOSE;
    double linkServoPosition = STEMperFiConstants.LINK_INIT;
    boolean iSpecimenMode = false;

    ButtonReader square2ButtonReader, triangle2ButtonReader, circle2ButtonReader, x2ButtonReader, dUp2ButtonReader, dDown2ButtonReader, dLeft2ButtonReader, dRight2ButtonReader, leftStick2ButtonReader, rightStick2ButtonReader;

    double armServoPosition = STEMperFiConstants.NEW_ARM_INIT;

    @Override
    public void runOpMode() {
        aprilTagInit();
        robotInit();
        while (opModeInInit()) {
            myOtos.resetTracking();
//            odo.resetPosAndIMU();
            telemetry.addData(">", "Robot Heading = %4.0f", Math.toDegrees(myOtos.getPosition().h));
            telemetry.update();
        }

       // driveStraight(0.2, 10, 0);
        turnToHeading(0.2, 45);
        sleep(10_000);
//        strafeLeft(.3,200);
//        //driveStraight(.3, 500, 0);
//        vMotor.setTargetPosition(STEMperFiConstants.SCORE_BUCKET_SPECIMEN);
//        vMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        vMotor.setPower(1);
//        sleep(1_000);
//        turnToDegrees(.3, -90);
//        ledServo.setPosition(STEMperFiConstants.GB_LED_GREEN);
//        sleep(500);
//        ledServo.setPosition(STEMperFiConstants.GB_LED_ORANGE);
//        driveStraight(.3, -180, 0);
//        sleep(500);
//        //strafeLeft(.3, 100);
//        strafeLeftTime(.5, 1_500);
//        vMotor.setTargetPosition(STEMperFiConstants.SCORE_BUCKET_SPECIMEN / 2);
//        vMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        vMotor.setPower(STEMperFiConstants.SCORE_BUCKET_DOWN_SPEED);
//        sleep(4_000);
//        strafeRight(.3, 200);
//        vMotor.setTargetPosition(10);
//        vMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        vMotor.setPower(1);
//        sleep(1_000);
//        moveToAnyTag();
//        vMotor.setPower(0);
//        turnToDegrees(.3, 75);
//        strafeRightTime(0.4, 1_500);

//        strafeLeft(.3, 200);
        //      strafeRight(.3, 200);
        // turnToDegrees(.2, -1.5708);
        // driveStraight(.2, -500, 0);
    }
    /*
     * Code to run ONCE when the driver hits INIT
     */

    public void aprilTagInit() {
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        // Initialize the Apriltag Detection process
        initAprilTag();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
//        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftfront_drive");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightfront_drive");
//        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftback_drive");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "rightback_drive");
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "fl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "bl");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br");

//        Motor frontLeft = new Motor(hardwareMap, "fl", Motor.GoBILDA.RPM_312);
//        Motor frontRight = new Motor(hardwareMap, "fr", Motor.GoBILDA.RPM_312);
//        Motor backLeft = new Motor(hardwareMap, "bl", Motor.GoBILDA.RPM_312);
//        Motor backRight = new Motor(hardwareMap, "br", Motor.GoBILDA.RPM_312);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips



        if (USE_WEBCAM) {
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        }
    }
    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder()
//                .setLensIntrinsics(481.985, 481.985, 334.203, 241.948)
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .setCameraResolution(new Size(640,480))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    public void moveToAnyTag() {
        double drive = 1;
        while (opModeIsActive() && drive > .1) {
            boolean targetFound = false;
            desiredTag  = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData("\n>","Tag Not Found\n");
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .

            if (targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                double turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                moveRobot(drive, strafe, turn);
            }
            telemetry.update();

            // Apply desired axes motions to the drivetrain.

            sleep(10);
        }

    }

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(-leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(-leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }


    public void robotInit() {
        gpA = new GamepadEx(gamepad1);
        gpB = new GamepadEx(gamepad2);
        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)  //  Rumble left motor 100% for 250 mSec
                .build();

//        // the extended gamepad object
//        gpA = new GamepadEx(gamepad1);
//        gpB = new GamepadEx(gamepad2);
//
//        square2ButtonReader = new ButtonReader(gpB, GamepadKeys.Button.X);
//        triangle2ButtonReader = new ButtonReader(gpB, GamepadKeys.Button.Y);
//        x2ButtonReader = new ButtonReader(gpB, GamepadKeys.Button.A);
//        circle2ButtonReader = new ButtonReader(gpB, GamepadKeys.Button.B);
//
//        dDown2ButtonReader = new ButtonReader(gpB, GamepadKeys.Button.DPAD_DOWN);
//        dUp2ButtonReader = new ButtonReader(gpB, GamepadKeys.Button.DPAD_UP);
//        dLeft2ButtonReader = new ButtonReader(gpB, GamepadKeys.Button.DPAD_LEFT);
//        dRight2ButtonReader = new ButtonReader(gpB, GamepadKeys.Button.DPAD_RIGHT);
//        rightStick2ButtonReader = new ButtonReader(gpB, GamepadKeys.Button.RIGHT_STICK_BUTTON);
//        leftStick2ButtonReader = new ButtonReader(gpB, GamepadKeys.Button.LEFT_STICK_BUTTON);

        bucketServo = hardwareMap.get(Servo.class, "bucket");
        bucketServo.setPosition(STEMperFiConstants.NEW_BUCKET_INIT);
        ledServo = hardwareMap.get(Servo.class, "gbled");
        ledServo.setPosition(STEMperFiConstants.GB_LED_WHITE);
        armServo = hardwareMap.get(Servo.class, "arm");
        armServo.setPosition(armServoPosition);
        pincherServo = hardwareMap.get(Servo.class, "intake");
        pincherServo.setPosition(pincherPosition);
        linkServo = hardwareMap.get(Servo.class, "link");
        linkServo.setPosition(linkServoPosition);

        Motor frontLeft = new Motor(hardwareMap, "fl", Motor.GoBILDA.RPM_312);
        Motor frontRight = new Motor(hardwareMap, "fr", Motor.GoBILDA.RPM_312);
        Motor backLeft = new Motor(hardwareMap, "bl", Motor.GoBILDA.RPM_312);
        Motor backRight = new Motor(hardwareMap, "br", Motor.GoBILDA.RPM_312);

        mecanum = new MecanumDrive(backRight, backLeft , frontRight, frontLeft);

       // hMotor = hardwareMap.get(DcMotor.class, "h");
       // hMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //hMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //hMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        vMotor = hardwareMap.get(DcMotor.class, "v");
        vMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        vMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vMotor.setTargetPosition(0);
        vMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vMotor.setPower(1);


        myOtos = myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        // All the configuration for the OTOS is done in this helper method, check it out!
        configureOtos();

    }



    /*
     * Code to run ONCE when the driver hits PLAY
     */
    public void noStart() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

         /*
     * Code to run ONCE after the driver hits STOP
     */

    public void noStop() {
    }

    // **********  HIGH Level driving functions.  ********************

    /**
     *  Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the OpMode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {
//        odo.resetPosAndIMU();
//        odo.update();
        SparkFunOTOS.Pose2D startPos = myOtos.getPosition();
        double targetX = startPos.x + distance;
        if (distance > 0) {
            mecanum.driveRobotCentric(0, maxDriveSpeed, 0);
            // Ensure that the OpMode is still active
            while (myOtos.getPosition().x < targetX && opModeIsActive()) {
                telemetry.addData("target x: ", targetX);
                telemetry.addData("current x: ", myOtos.getPosition().x);
                telemetry.update();
            }
        } else {
            mecanum.driveRobotCentric(0, -maxDriveSpeed, 0);
            // Ensure that the OpMode is still active
            while (myOtos.getPosition().x > targetX && opModeIsActive()) {
                telemetry.addData("target x: ", targetX);
                telemetry.addData("current x: ", myOtos.getPosition().x);
                telemetry.update();
            }
        }
        mecanum.driveRobotCentric(0, 0, 0);
    }

    public void turnToHeading(double maxTurnSpeed, double targetHeading) {
        double currentHeading = myOtos.getPosition().h;
        if (targetHeading > currentHeading) {
            mecanum.driveRobotCentric(0, 0, maxTurnSpeed);
            while (myOtos.getPosition().h < targetHeading && opModeIsActive()) {
                ledServo.setPosition(STEMperFiConstants.GB_LED_RED);
                telemetry.addData("target h:", targetHeading);
                telemetry.addData("current h:", myOtos.getPosition().h);
                telemetry.update();
                sleep(10);
            }
        } else {
            mecanum.driveRobotCentric(0, 0, -maxTurnSpeed);
            while (myOtos.getPosition().h > targetHeading && opModeIsActive()) {
                ledServo.setPosition(STEMperFiConstants.GB_LED_BLUE);
                telemetry.addData("target h:", targetHeading);
                telemetry.addData("current h:", myOtos.getPosition().h);
                telemetry.update();
                sleep(10);
            }
        }
        ledServo.setPosition(STEMperFiConstants.GB_LED_OFF);
        mecanum.driveRobotCentric(0, 0, 0);
    }


    public void turnToDegrees(double maxTurnSpeed, double angDeg) {
//        odo.update();
//        if (angDeg > 0) {
//            mecanum.driveRobotCentric(0, 0, maxTurnSpeed);
//            while (Math.toDegrees(odo.getHeading()) < angDeg && opModeIsActive()) {
//                double degrees = Math.toDegrees(odo.getHeading());
//                ledServo.setPosition(STEMperFiConstants.GB_LED_RED);
//                telemetry.addData("target:", angDeg);
//                telemetry.addData("HR:", odo.getHeading());
//                telemetry.addData("Heading: ", degrees);
//                telemetry.addData("pos", odo.getPosition());
//                telemetry.addData("pos x", odo.getPosX());
//                telemetry.addData("pos y", odo.getPosY());
//                telemetry.update();
//                sleep(10);
//                odo.update();
//            }
//        } else {
//            mecanum.driveRobotCentric(0, 0, -maxTurnSpeed);
//            while (Math.toDegrees(odo.getHeading()) > angDeg && opModeIsActive()) {
//                ledServo.setPosition(STEMperFiConstants.GB_LED_BLUE);
//                double degrees = Math.toDegrees(odo.getHeading());
//                telemetry.addData("target:", angDeg);
//                telemetry.addData("HR:", odo.getHeading());
//                telemetry.addData("Heading: ", degrees);
//                telemetry.addData("pos", odo.getPosition());
//                telemetry.addData("pos x", odo.getPosX());
//                telemetry.addData("pos y", odo.getPosY());
//                telemetry.update();
//                sleep(10);
//                odo.update();
//            }
//        }
//        ledServo.setPosition(STEMperFiConstants.GB_LED_OFF);
//        mecanum.driveRobotCentric(0, 0, 0);
    }

    public void strafeLeftTime(double maxDriveSpeed,
                               long timeMs) {
//        long start = System.currentTimeMillis();
//        odo.resetPosAndIMU();
//        odo.update();
//        mecanum.driveRobotCentric(-maxDriveSpeed, 0, 0);
//        // Ensure that the OpMode is still active
//        while (opModeIsActive() && ((System.currentTimeMillis() - start) < timeMs)) {
//            sleep(100);
//            odo.update();
//            telemetry.addData("current y: ", odo.getPosY());
//            telemetry.update();
//        }
//
//        mecanum.driveRobotCentric(0, 0, 0);
    }

    public void strafeLeft(double maxDriveSpeed,
                           double distance) {
//        odo.resetPosAndIMU();
//        odo.update();
//        mecanum.driveRobotCentric(-maxDriveSpeed, 0, 0);
//        // Ensure that the OpMode is still active
//        while (odo.getPosY() < distance && opModeIsActive()) {
//            odo.update();
//            telemetry.addData("target y: ", distance);
//            telemetry.addData("current y: ", odo.getPosY());
//            telemetry.update();
//        }
//
//        mecanum.driveRobotCentric(0, 0, 0);
    }

    public void strafeRight(double maxDriveSpeed,
                            double distance) {
//        odo.resetPosAndIMU();
//        odo.update();
//        mecanum.driveRobotCentric(maxDriveSpeed, 0, 0);
//        // Ensure that the OpMode is still active
//        while (odo.getPosY() > -distance && opModeIsActive()) {
//            odo.update();
//            telemetry.addData("target y: ", distance);
//            telemetry.addData("current y: ", odo.getPosY());
//            telemetry.update();
//        }
//
//        mecanum.driveRobotCentric(0, 0, 0);
    }

    public void strafeRightTime(double maxDriveSpeed, long timeMs) {
//        long start = System.currentTimeMillis();
//        odo.resetPosAndIMU();
//        odo.update();
//        mecanum.driveRobotCentric(maxDriveSpeed, 0, 0);
//        // Ensure that the OpMode is still active
//        while (opModeIsActive() && ((System.currentTimeMillis() - start) < timeMs)) {
//            sleep(100);
//            odo.update();
//            telemetry.addData("current y: ", odo.getPosY());
//            telemetry.update();
//        }
//
//        mecanum.driveRobotCentric(0, 0, 0);
    }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0.75, -2.86, 0);
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        myOtos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        myOtos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }



}
