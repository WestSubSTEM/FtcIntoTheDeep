package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

@Autonomous(name="Sample Auto", group="Qual")
//@Disabled
public class SampleAuto extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    Pose3D robotPoseTag = null;

    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;
    DcMotor leftBackDrive;
    DcMotor rightBackDrive;

    // Robot components
    Servo bucketServo, ledServo, armServo, pincherServo, hangServo;
    double pincherPosition = STEMperFiConstants.PINCHER_OPEN;
    DcMotor hMotor, vMotor;
    double armServoPosition = STEMperFiConstants.ARM_INIT;

    private final ElapsedTime stateElapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class

    enum StateMachine {
        WAITING_FOR_START,
        PULL_AWAY_FROM_WALL,
        DRIVE_TO_TAG_IN_START,
        DRIVE_TO_BASKET_START,
        DRIVE_TO_TAG_OUT_START,
        DRIVE_TO_SAMPLE_1,
        DRIVE_TO_TAG_IN_1,
        DRIVE_TO_BASKET_1,
        DRIVE_TO_TAG_OUT_1,
        DRIVE_TO_SAMPLE_2,
        DRIVE_TO_TAG_IN_2,
        DRIVE_TO_BASKET_2,
        PRE_PARK,
        PARK,
        END
    }


    /**
    static final Pose2D TARGET_1 = new Pose2D(DistanceUnit.MM,2000 / 10,20,AngleUnit.DEGREES,0);
    static final Pose2D TARGET_2 = new Pose2D(DistanceUnit.MM, 2600 /10, -20, AngleUnit.DEGREES, -90);
    static final Pose2D TARGET_3 = new Pose2D(DistanceUnit.M M,2600 / 10,-2600 / 10, AngleUnit.DEGREES,-90);
    static final Pose2D TARGET_4 = new Pose2D(DistanceUnit.MM, 100, -2600/10, AngleUnit.DEGREES, 90);
    static final Pose2D TARGET_5 = new Pose2D(DistanceUnit.MM, 100, 0, AngleUnit.DEGREES, 0);
*/
    static final Pose2D TARGET_0_WALL_BLUE = new Pose2D(DistanceUnit.INCH, 36, 60, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_1_WALL_AWAY_BLUE = new Pose2D(DistanceUnit.INCH, 36, 39, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_2_TAG_IN_START_BLUE = new Pose2D(DistanceUnit.INCH, 45, 39, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_3_BASKET_START_BLUE = new Pose2D(DistanceUnit.INCH, 63.4025, 55.9483, AngleUnit.DEGREES, -25);
    static final Pose2D TARGET_4_TAG_OUT_START_BLUE = TARGET_2_TAG_IN_START_BLUE;
    // SAMPLE 1 POSITION !!!!!!!!!!!!!!!!!!!!!!!!
    // BIGGER X CLOSER TO WALL, SMALLER X CLOSER TO SUB, BIGGER Y CLOSER TO DRIVERS, SMALLER Y CLOSER TO SUB
    static final Pose2D TARGET_5_SAMPLE_1_BLUE = new Pose2D(DistanceUnit.INCH, 38.3, 24.75, AngleUnit.DEGREES, 0); // hmotor 304
    static final Pose2D TARGET_6_TAG_IN_1_BLUE = TARGET_2_TAG_IN_START_BLUE;
    static final Pose2D TARGET_7_BASKET_1_BLUE = TARGET_3_BASKET_START_BLUE;
    static final Pose2D TARGET_8_TAG_OUT_1_BLUE = TARGET_2_TAG_IN_START_BLUE;
    // SAMPLE 2 POSITION !!!!!!!!!!!!!!!!!!!!!!!!
    static final Pose2D TARGET_9_SAMPLE_2_BLUE = new Pose2D(DistanceUnit.INCH, 48 , 24.5, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_10_TAG_IN_2_BLUE = TARGET_2_TAG_IN_START_BLUE;
    static final Pose2D TARGET_11_BASKET_2_BLUE = TARGET_3_BASKET_START_BLUE;
//    static final Pose2D TARGET_12_TAG_OUT_2_BLUE = TARGET_2_TAG_IN_START_BLUE;
//    static final Pose2D TARGET_13_SAMPLE_3_BLUE = new Pose2D(DistanceUnit.INCH, 37.8013 + 20, 24.5, AngleUnit.DEGREES, 0);
//    static final Pose2D TARGET_14_TAG_IN_3_BLUE = TARGET_2_TAG_IN_START_BLUE;
//    static final Pose2D TARGET_15_BASKET_3_BLUE = TARGET_3_BASKET_START_BLUE;
//    static final Pose2D TARGET_16_TAG_OUT_3_BLUE = TARGET_2_TAG_IN_START_BLUE;
static final Pose2D TARGET_12_PRE_PARK_BLUE = new Pose2D(DistanceUnit.INCH, 40, 6.5, AngleUnit.DEGREES, -90);

    static final Pose2D TARGET_13_PARK_BLUE = new Pose2D(DistanceUnit.INCH, 27, 6.5, AngleUnit.DEGREES, -90);

    static final Pose2D[] BLUE_TARGETS = {
            TARGET_0_WALL_BLUE,
            TARGET_1_WALL_AWAY_BLUE,
            TARGET_2_TAG_IN_START_BLUE, TARGET_3_BASKET_START_BLUE, TARGET_4_TAG_OUT_START_BLUE,
            TARGET_5_SAMPLE_1_BLUE, TARGET_6_TAG_IN_1_BLUE, TARGET_7_BASKET_1_BLUE, TARGET_8_TAG_OUT_1_BLUE,
            TARGET_9_SAMPLE_2_BLUE, TARGET_10_TAG_IN_2_BLUE, TARGET_11_BASKET_2_BLUE,
            //TARGET_13_SAMPLE_3_BLUE, TARGET_14_TAG_IN_3_BLUE, TARGET_15_BASKET_3_BLUE, TARGET_16_TAG_OUT_3_BLUE,
            TARGET_12_PRE_PARK_BLUE, TARGET_13_PARK_BLUE};

    Pose2D[] targets = BLUE_TARGETS;
    StateMachine stateMachine;
    @Override
    public void runOpMode() {
        initRobot();

        //nav.setXYCoefficients(0.02,0.002,0.0,DistanceUnit.MM,12);
        //nav.setYawCoefficients(1,0,0.0, AngleUnit.DEGREES,2);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        odo.setPosition(targets[0]);

        // Wait for the game to start (driver presses START)
        ledServo.setPosition(STEMperFiConstants.GB_LED_GREEN);
        waitForStart();
        resetRuntime();
        setState(StateMachine.WAITING_FOR_START);
        hangServo.setPosition(STEMperFiConstants.LEVEL_ONE_HANG_DRIVE);

//        if (false) {
//            bucketServo.setPosition(STEMperFiConstants.BUCKET_INTAKE);
//            sleep(1_000);
//            getSample();
//            stateMachine = StateMachine.END;
//        }
        while (opModeIsActive()) {
            odo.update();

            switch (stateMachine){
                case WAITING_FOR_START:
                    odo.setPosition(targets[0]);
                    //the first step in the autonomous
                    setState(StateMachine.PULL_AWAY_FROM_WALL);
                    break;
                case PULL_AWAY_FROM_WALL:
                    if (nav.driveTo(odo.getPosition(), targets[1], .6, .1)){
                        telemetry.addLine("at " + stateMachine);
                        setState(StateMachine.DRIVE_TO_TAG_IN_START);
                        raiseBucket();
                    }
                    break;
                case DRIVE_TO_TAG_IN_START:
                    if (nav.driveTo(odo.getPosition(), targets[2], 0.6, .1)){
                        telemetry.addLine("at " + stateMachine);
                        bucketServo.setPosition(STEMperFiConstants.BUCKET_AUTO_TO_SCORE);
                        setState(StateMachine.DRIVE_TO_BASKET_START);
                    }
                    break;
                case DRIVE_TO_BASKET_START:
                    if(nav.driveTo(odo.getPosition(), targets[3], 0.6, .1)){
                        telemetry.addLine("at " + stateMachine);
                        score();
                        setState(StateMachine.DRIVE_TO_TAG_OUT_START);
                    }
                    break;
                case DRIVE_TO_TAG_OUT_START:
                    if (nav.driveTo(odo.getPosition(), targets[4], 0.6, .1)){
                        telemetry.addLine("at " + stateMachine);
                        lowerBasket();
                        setState(StateMachine.DRIVE_TO_SAMPLE_1);
                    }
                    break;
                case DRIVE_TO_SAMPLE_1:
                     if(nav.driveTo(odo.getPosition(),targets[5],0.6, .1)){
                        telemetry.addLine("at " + stateMachine);
                        getSample();
                        raiseBucket();
                        setState(StateMachine.DRIVE_TO_TAG_IN_1);
                    }
                    break;
                case DRIVE_TO_TAG_IN_1:
                    if(nav.driveTo(odo.getPosition(),targets[6],0.6,.1)){
                        telemetry.addLine("at " + stateMachine);
                        bucketServo.setPosition(STEMperFiConstants.BUCKET_AUTO_TO_SCORE);
                        setState(StateMachine.DRIVE_TO_BASKET_1);
                    }
                    break;
                case DRIVE_TO_BASKET_1:
                    if(nav.driveTo(odo.getPosition(),targets[7],0.6,.1)){
                        telemetry.addLine("at " + stateMachine);
                        score();
                        setState(StateMachine.DRIVE_TO_TAG_OUT_1);
                    }
                    break;
                case DRIVE_TO_TAG_OUT_1:
                    if(nav.driveTo(odo.getPosition(),targets[8],0.6,.1)){
                        telemetry.addLine("at " + stateMachine);
                        lowerBasket();
                        setState(StateMachine.DRIVE_TO_SAMPLE_2);
                    }
                    break;
                case DRIVE_TO_SAMPLE_2:
                    if(nav.driveTo(odo.getPosition(),targets[9],0.6,.1)){
                        telemetry.addLine("at " + stateMachine);
                        getSample();
                        raiseBucket();
                        setState(StateMachine.DRIVE_TO_TAG_IN_2);
                    }
                    break;
                case DRIVE_TO_TAG_IN_2:
                    if(nav.driveTo(odo.getPosition(),targets[10],0.6,.1)){
                        telemetry.addLine("at " + stateMachine);
                        bucketServo.setPosition(STEMperFiConstants.BUCKET_AUTO_TO_SCORE);
                        setState(StateMachine.DRIVE_TO_BASKET_2);
                    }
                    break;
                case DRIVE_TO_BASKET_2:
                    if(nav.driveTo(odo.getPosition(),targets[11],0.6,.1)){
                        telemetry.addLine("at " + stateMachine);
                        score();
                        setState(StateMachine.PRE_PARK);
                    }
                    break;
                case PRE_PARK:
                    if(nav.driveTo(odo.getPosition(),targets[12],0.8,.1)){
                        telemetry.addLine("at " + stateMachine);
                        turnOffVerticalLift();
                        setState(StateMachine.PARK);
                    }
                    if (stateElapsedTime.time() > 250) {
                        lowerBasket();
                        backToInit();
                    }
                    break;
                case PARK:
                    if(nav.driveTo(odo.getPosition(),targets[13],0.5,.1)){
                        telemetry.addLine("at " + stateMachine);
                        setState(StateMachine.END);
                        hangServo.setPosition(STEMperFiConstants.LEVEL_ONE_HANG_HANG);
                    }
                    break;

            }


            //nav calculates the power to set to each motor in a mecanum or tank drive. Use nav.getMotorPower to find that value.
            leftFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
            rightFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
            leftBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
            rightBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

            telemetry.addData("current state:",stateMachine);

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            telemetry.update();

        }
    }

    private void setState(StateMachine newState) {
        stateElapsedTime.reset();
        stateMachine = newState;
    }


    private void getSample() {
        ledServo.setPosition(STEMperFiConstants.GB_LED_RED);
        armServo.setPosition(STEMperFiConstants.ARM_AIM);
        pincherServo.setPosition(STEMperFiConstants.PINCHER_OPEN);

        hMotor.setTargetPosition(1_000);
        hMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hMotor.setPower(1);
        ledServo.setPosition(STEMperFiConstants.GB_LED_BLUE);
        while(hMotor.isBusy() && opModeIsActive()) {
            sleep(10);
        }
        ledServo.setPosition(STEMperFiConstants.GB_LED_GREEN);
        armServo.setPosition(STEMperFiConstants.ARM_INTAKE);
        sleep(500);
        pincherServo.setPosition(STEMperFiConstants.PINCHER_CLOSE);
        sleep(500);
        armServo.setPosition(STEMperFiConstants.ARM_SCORE);

        ledServo.setPosition(STEMperFiConstants.GB_LED_BLUE);
        hMotor.setTargetPosition(STEMperFiConstants.H_SCORE);
        hMotor.setPower(1);
        while(hMotor.isBusy() && opModeIsActive()) {
            sleep(10);
        }
        ledServo.setPosition(STEMperFiConstants.GB_LED_GREEN);

        turnOffVerticalLift();
        sleep(1_500);
        pincherServo.setPosition(STEMperFiConstants.PINCHER_OPEN);
        sleep(250);
        armServo.setPosition(STEMperFiConstants.ARM_INIT);
        sleep(250);
        vMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void raiseBucket() {
        hMotor.setTargetPosition(STEMperFiConstants.H_SCORE);
        hMotor.setPower(1);
        bucketServo.setPosition(STEMperFiConstants.BUCKET_INTAKE);
        vMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vMotor.setTargetPosition(STEMperFiConstants.SCORE_BUCKET_SECOND);
        vMotor.setPower(1);
        sleep(500);
    }

    private void score() {
        bucketServo.setPosition(STEMperFiConstants.BUCKET_SCORE);
        sleep(900);
        bucketServo.setPosition(STEMperFiConstants.BUCKET_INTAKE);
    }

    private void backToInit() {
        hMotor.setTargetPosition(0);
        hMotor.setPower(1);
        bucketServo.setPosition(STEMperFiConstants.BUCKET_INIT);
        armServo.setPosition(STEMperFiConstants.ARM_INIT);
    }

    private void lowerBasket() {
        vMotor.setTargetPosition(0);
        vMotor.setPower(1);
        pincherServo.setPosition(STEMperFiConstants.PINCHER_OPEN);
        armServo.setPosition(STEMperFiConstants.ARM_AIM);
    }

    private void turnOffVerticalLift() {
        vMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        vMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vMotor.setPower(0);
    }

    void initRobot() {
        hangServo = hardwareMap.get(Servo.class, "hang");
        hangServo.setPosition(STEMperFiConstants.LEVEL_ONE_HANG_INIT);
        bucketServo = hardwareMap.get(Servo.class, "bucket");
        bucketServo.setPosition(STEMperFiConstants.BUCKET_INIT);
        ledServo = hardwareMap.get(Servo.class, "gbled");
        ledServo.setPosition(STEMperFiConstants.GB_LED_WHITE);
        armServo = hardwareMap.get(Servo.class, "arm");
        armServo.setPosition(armServoPosition);
        pincherServo = hardwareMap.get(Servo.class, "intake");
        pincherServo.setPosition(pincherPosition);

        hMotor = hardwareMap.get(DcMotor.class, "h");

        hMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        hMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hMotor.setTargetPosition(0);
        hMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hMotor.setPower(1);

        vMotor = hardwareMap.get(DcMotor.class, "v");
        vMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        vMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vMotor.setTargetPosition(0);
        vMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vMotor.setPower(1);

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "fl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "bl");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "br");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odo.setOffsets(-84.0, -168.0); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        odo.recalibrateIMU();
        odo.resetPosAndIMU();
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
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640,480))
                .addProcessor(aprilTag)
                .build();

        if (USE_WEBCAM) {
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        }

    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void  setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while ((visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                try {
                    Thread.sleep(20);
                } catch(Exception e) {
                    telemetry.addLine(e.getMessage());
                    telemetry.update();
                }
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
        if (true)  {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                try {
                    Thread.sleep(50);
                } catch(Exception e) {
                    telemetry.addLine(e.getMessage());
                    telemetry.update();
                }
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            try {
                Thread.sleep(20);
            } catch(Exception e) {
                telemetry.addLine(e.getMessage());
                telemetry.update();
            }
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            try {
                Thread.sleep(20);
            } catch(Exception e) {
                telemetry.addLine(e.getMessage());
                telemetry.update();
            }
        }
    }

    public void detectTag() {
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
                    robotPoseTag = desiredTag.robotPose;
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

    }

}