package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
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
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="Tele Tag", group="FTC Lib")
public class TeleTag extends OpMode
{
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag


    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    Gamepad.RumbleEffect customRumbleEffect;    // Use to build a custom rumble sequence.

    double oldTime = 0;
    int bucketVerticalPosition = 0;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor hMotor, vMotor;
    // input motors exactly as shown below
    MecanumDrive mecanum;
    GamepadEx gpA, gpB;

    long attackStartTime =  0;
    long liftStartTime = 0;
    long odoResetTime = 0;

    Servo bucketServo, ledServo, armServo, pincherServo;
    double pincherPosition = STEMperFiConstants.PINCHER_OPEN;

    boolean iSpecimenMode = false;

    Pose3D robotPoseTag = null;
    static final Pose2D TARGET_0_WALL_BLUE = new Pose2D(DistanceUnit.INCH, 36, 60, AngleUnit.DEGREES, 0);

    ButtonReader startButtonReader, square2ButtonReader, triangle2ButtonReader, circle2ButtonReader, x2ButtonReader, dUp2ButtonReader, dDown2ButtonReader, dLeft2ButtonReader, dRight2ButtonReader, leftStick2ButtonReader, rightStick2ButtonReader;

    double armServoPosition = STEMperFiConstants.ARM_INIT;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)  //  Rumble left motor 100% for 250 mSec
                .build();

        // the extended gamepad object
        gpA = new GamepadEx(gamepad1);
        gpB = new GamepadEx(gamepad2);

        startButtonReader = new ButtonReader(gpB, GamepadKeys.Button.START);
        square2ButtonReader = new ButtonReader(gpB, GamepadKeys.Button.X);
        triangle2ButtonReader = new ButtonReader(gpB, GamepadKeys.Button.Y);
        x2ButtonReader = new ButtonReader(gpB, GamepadKeys.Button.A);
        circle2ButtonReader = new ButtonReader(gpB, GamepadKeys.Button.B);

        dDown2ButtonReader = new ButtonReader(gpB, GamepadKeys.Button.DPAD_DOWN);
        dUp2ButtonReader = new ButtonReader(gpB, GamepadKeys.Button.DPAD_UP);
        dLeft2ButtonReader = new ButtonReader(gpB, GamepadKeys.Button.DPAD_LEFT);
        dRight2ButtonReader = new ButtonReader(gpB, GamepadKeys.Button.DPAD_RIGHT);
        rightStick2ButtonReader = new ButtonReader(gpB, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        leftStick2ButtonReader = new ButtonReader(gpB, GamepadKeys.Button.LEFT_STICK_BUTTON);

        bucketServo = hardwareMap.get(Servo.class, "bucket");
        bucketServo.setPosition(STEMperFiConstants.BUCKET_INIT);
        ledServo = hardwareMap.get(Servo.class, "gbled");
        ledServo.setPosition(STEMperFiConstants.GB_LED_WHITE);
        armServo = hardwareMap.get(Servo.class, "arm");
        armServo.setPosition(armServoPosition);
        pincherServo = hardwareMap.get(Servo.class, "intake");
        pincherServo.setPosition(pincherPosition);

        Motor frontLeft = new Motor(hardwareMap, "fl", Motor.GoBILDA.RPM_312);
        Motor frontRight = new Motor(hardwareMap, "fr", Motor.GoBILDA.RPM_312);
        Motor backLeft = new Motor(hardwareMap, "bl", Motor.GoBILDA.RPM_312);
        Motor backRight = new Motor(hardwareMap, "br", Motor.GoBILDA.RPM_312);

        mecanum = new MecanumDrive(backRight, backLeft , frontRight, frontLeft);

        hMotor = hardwareMap.get(DcMotor.class, "h");
        hMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        hMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        vMotor = hardwareMap.get(DcMotor.class, "v");
        vMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        vMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vMotor.setTargetPosition(0);
        vMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vMotor.setPower(1);

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

        initAprilTag();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        odo.setPosition(TARGET_0_WALL_BLUE);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        odo.setPosition(TARGET_0_WALL_BLUE);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        odo.update();
        double lx = gpA.getLeftX();
        double ly = gpA.getLeftY();
        double rx = gpA.getRightX();
        if (gamepad1.left_bumper || vMotor.getCurrentPosition() > 400) {
            lx = lx / 2;
            ly = ly / 2;
            rx = rx / 2;
        }
        double degrees = Math.toDegrees(odo.getHeading());

        mecanum.driveFieldCentric(
                lx,
                ly,
                rx,
                degrees,   // gyro value passed in here must be in degrees
                false
        );

        // Gamepad 1
        if (gamepad1.right_trigger > .9 && gamepad1.left_trigger > .9) {
            long now = System.currentTimeMillis();
            if (now - odoResetTime > 1_000) {
                odo.resetPosAndIMU();
                odoResetTime = now;
                gamepad1.runRumbleEffect(customRumbleEffect);

                //ledServo.setPosition(STEMperFiConstants.GB_LED_YELLOW);
            }
        }
        Pose2D pos = odo.getPosition();
        telemetry.addData("pos x in: ", pos.getX(DistanceUnit.INCH));
        telemetry.addData("pos y in: ", pos.getY(DistanceUnit.INCH));
        telemetry.addData("head deg: ", pos.getHeading(AngleUnit.DEGREES));
        // Detect Tag
        detectTag();
        if (robotPoseTag != null) {
            telemetry.addData("tag robot x in: ", robotPoseTag.getPosition().toUnit(DistanceUnit.INCH).x);
            telemetry.addData("tag robot y in: ", robotPoseTag.getPosition().toUnit(DistanceUnit.INCH).y);
            telemetry.addData("tag robot head deg: ", robotPoseTag.getOrientation().getYaw(AngleUnit.DEGREES));

            if (ledServo.getPosition() != STEMperFiConstants.GB_LED_GREEN) {
                ledServo.setPosition(STEMperFiConstants.GB_LED_RED);
            }
        } else {
            ledServo.setPosition(STEMperFiConstants.GB_LED_OFF);
        }
        startButtonReader.readValue();
        if (robotPoseTag != null && startButtonReader.wasJustPressed()) {
            odo.setPosition(new Pose2D(
                    DistanceUnit.INCH,
                    robotPoseTag.getPosition().toUnit(DistanceUnit.INCH).x,
                    robotPoseTag.getPosition().toUnit(DistanceUnit.INCH).y,
                    AngleUnit.RADIANS,
                    robotPoseTag.getOrientation().getYaw(AngleUnit.RADIANS))
            );
            ledServo.setPosition(STEMperFiConstants.GB_LED_GREEN);
        }
        // Gamepad 2
        x2ButtonReader.readValue();
        circle2ButtonReader.readValue();
        square2ButtonReader.readValue();
        triangle2ButtonReader.readValue();

        dUp2ButtonReader.readValue();
        dDown2ButtonReader.readValue();
        dLeft2ButtonReader.readValue();
        dRight2ButtonReader.readValue();
        rightStick2ButtonReader.readValue();
        leftStick2ButtonReader.readValue();

        if (dRight2ButtonReader.wasJustPressed()) {
            this.iSpecimenMode = !this.iSpecimenMode;
            if (this.iSpecimenMode) {
                pincherPosition = STEMperFiConstants.PINCHER_CLOSE;
                armServoPosition = STEMperFiConstants.ARM_INIT;
                bucketServo.setPosition(STEMperFiConstants.BUCKET_INIT);
                //ledServo.setPosition(STEMperFiConstants.GB_LED_INDIGO);
                hMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hMotor.setTargetPosition(STEMperFiConstants.H_SCORE);
                hMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hMotor.setPower(1);
            } else {
                //ledServo.setPosition(STEMperFiConstants.GB_LED_OFF);
            }
        }

        if (iSpecimenMode) {
            if (triangle2ButtonReader.wasJustPressed()) {
                attackStartTime = 0;
                pincherPosition = STEMperFiConstants.PINCHER_OPEN;
                armServoPosition = STEMperFiConstants.ARM_AIM;
                //ledServo.setPosition(STEMperFiConstants.GB_LED_RED);
            } else if (x2ButtonReader.wasJustPressed() || dLeft2ButtonReader.wasJustPressed()) {
                attackStartTime = 0;
                armServoPosition = STEMperFiConstants.ARM_DRIVE;
                hMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hMotor.setTargetPosition(STEMperFiConstants.H_SCORE);
                hMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hMotor.setPower(1);
            } else if (circle2ButtonReader.wasJustPressed()) {
                attackStartTime = System.currentTimeMillis();
                armServoPosition = STEMperFiConstants.ARM_INTAKE;
                //ledServo.setPosition(STEMperFiConstants.GB_LED_GREEN);
            } else if (square2ButtonReader.wasJustPressed()) {
                attackStartTime = 0;
                armServoPosition = STEMperFiConstants.ARM_DRIVE;
                //ledServo.setPosition(STEMperFiConstants.GB_LED_ORANGE);
            }
        } else {
            if (triangle2ButtonReader.wasJustPressed()) {
                attackStartTime = 0;
                pincherPosition = STEMperFiConstants.PINCHER_OPEN;
                armServoPosition = STEMperFiConstants.ARM_AIM;
                //ledServo.setPosition(STEMperFiConstants.GB_LED_RED);
            } else if (x2ButtonReader.wasJustPressed()) {
                attackStartTime = 0;
                armServoPosition = STEMperFiConstants.ARM_SCORE;
                hMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hMotor.setTargetPosition(STEMperFiConstants.H_SCORE);
                hMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hMotor.setPower(1);
            } else if (circle2ButtonReader.wasJustPressed()) {
                attackStartTime = System.currentTimeMillis();
                armServoPosition = STEMperFiConstants.ARM_INTAKE;
                //ledServo.setPosition(STEMperFiConstants.GB_LED_GREEN);
            } else if (square2ButtonReader.wasJustPressed()) {
                attackStartTime = 0;
                armServoPosition = STEMperFiConstants.ARM_DRIVE;
                ledServo.setPosition(STEMperFiConstants.GB_LED_ORANGE);
            }
        }
        armServo.setPosition(armServoPosition);
        if (attackStartTime > 0) {
            long attackDuration = System.currentTimeMillis() - attackStartTime;
            if (attackDuration > STEMperFiConstants.ATTACK_DRIVE_ms) {
                attackStartTime = 0;
                armServoPosition = STEMperFiConstants.ARM_DRIVE;
                //ledServo.setPosition(STEMperFiConstants.GB_LED_ORANGE);
                armServo.setPosition(armServoPosition);
            } else if (attackDuration > STEMperFiConstants.ATTACK_PINCHER_CLOSE_ms) {
                pincherPosition = STEMperFiConstants.PINCHER_CLOSE;
                armServo.setPosition(armServoPosition);
            }
        }

        if (gamepad2.left_bumper && attackStartTime == 0) {
            pincherPosition = STEMperFiConstants.PINCHER_OPEN;
        } else if (gamepad2.right_bumper) {
            pincherPosition = STEMperFiConstants.PINCHER_CLOSE;
        }
        pincherServo.setPosition(pincherPosition);


        if (!this.iSpecimenMode) {
            double leftTrigger = gpB.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
            double rightTrigger = gpB.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            if (rightTrigger > 0.1 || leftTrigger > 0.1) {
                bucketServo.setPosition(STEMperFiConstants.BUCKET_SCORE);
            } else {
                bucketServo.setPosition(STEMperFiConstants.BUCKET_INTAKE);
            }
        }

        double gp2LY = gpB.getLeftY();
        int vTargetDiff = vMotor.getCurrentPosition() - vMotor.getTargetPosition();
        if (leftStick2ButtonReader.wasJustPressed()) {
            vMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bucketVerticalPosition = 0;
        } else if (dDown2ButtonReader.wasJustPressed()) {
            bucketVerticalPosition = 0;
            vMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } else if (dUp2ButtonReader.wasJustPressed()) {
            armServoPosition = iSpecimenMode ? STEMperFiConstants.ARM_INIT : STEMperFiConstants.ARM_INIT_SAMPLE;
            armServo.setPosition(armServoPosition);
            vMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (bucketVerticalPosition == STEMperFiConstants.SCORE_BUCKET_FIRST) {
                bucketVerticalPosition = STEMperFiConstants.SCORE_BUCKET_SECOND;
            } else {
                liftStartTime = System.currentTimeMillis();
                bucketVerticalPosition = iSpecimenMode ? STEMperFiConstants.SCORE_BUCKET_SPECIMEN : STEMperFiConstants.SCORE_BUCKET_FIRST;
            }
        } else if (Math.abs(gp2LY) > 0.1 && Math.abs(vTargetDiff) < 100) {
            vMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bucketVerticalPosition += Math.round(gp2LY * 100);
            bucketVerticalPosition = Math.max(-50, bucketVerticalPosition);
        }
        if (iSpecimenMode) {
            vMotor.setTargetPosition(bucketVerticalPosition);
            vMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (bucketVerticalPosition > vMotor.getCurrentPosition()) {
                vMotor.setPower(1);
            } else {
                vMotor.setPower(STEMperFiConstants.SCORE_BUCKET_DOWN_SPEED);
            }
        } else if (System.currentTimeMillis() - liftStartTime > STEMperFiConstants.LIFT_DELAY_MS) {
            vMotor.setTargetPosition(bucketVerticalPosition);
            vMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            vMotor.setPower(1);
        }
        if (bucketVerticalPosition == 0 && vTargetDiff < 50) {
            vMotor.setPower(0);
        }

//        telemetry.addData("Left X", lx);
//        telemetry.addData("Left Y", ly);
//        telemetry.addData("Right X", rx);
        telemetry.addData("HR:", odo.getHeading());
        telemetry.addData("Heading: ", degrees);
        telemetry.addData("pos", odo.getPosition());
        telemetry.addData("pos x", odo.getPosX());
        telemetry.addData("pos y", odo.getPosY());

        double gp2RY = -gpB.getRightY();
        if (dRight2ButtonReader.wasJustPressed()) {
            hMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hMotor.setTargetPosition(STEMperFiConstants.H_SCORE);
            hMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hMotor.setPower(1);
        } else if (rightStick2ButtonReader.wasJustPressed()) {
            hMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else {
            if (hMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION || (Math.abs(gp2RY) > 0.1)) {
                hMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                hMotor.setPower(gp2RY);
            }
        }
        telemetry.addData("v", gp2LY);
        telemetry.addData("targetPosition", bucketVerticalPosition);
        telemetry.addData("vMotor.getTargetPosition)", vMotor.getTargetPosition());
        telemetry.addData("vMotor.getCurrentPosition()", vMotor.getCurrentPosition());

        //vMotor.set(gp2LY);
        telemetry.addData("h", gp2RY);
        telemetry.addData("hMotor.getCurrentPosition()", hMotor.getCurrentPosition());
        telemetry.addData("hMotor.getTargetPosition)", hMotor.getTargetPosition());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
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
