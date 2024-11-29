package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Tele Meet 2", group="FTC Lib")
public class TeleMeet1 extends OpMode
{
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    double oldTime = 0;
    int bucketVerticalPosition = 0;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor hMotor, vMotor;
    // input motors exactly as shown below
    MecanumDrive mecanum;
    GamepadEx gpA, gpB;
    GamepadButton leftBumper, rightBumper;

    long attackStartTime =  0;
    long liftStartTime = 0;

    Servo bucketServo, ledServo, armServo, pincherServo;
    double pincherPosition = STEMperFiConstants.PINCHER_OPEN;

    ButtonReader square2ButtonReader, triangle2ButtonReader, circle2ButtonReader, x2ButtonReader, dUp2ButtonReader, dDown2ButtonReader, dLeft2ButtonReader, dRight2ButtonReader, leftStick2ButtonReader, rightStick2ButtonReader;

    double armServoPosition = STEMperFiConstants.ARM_INIT;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // the extended gamepad object
        gpA = new GamepadEx(gamepad1);
        gpB = new GamepadEx(gamepad2);

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
        bucketServo.setPosition(STEMperFiConstants.BUCKET_INTAKE);
        ledServo = hardwareMap.get(Servo.class, "gbled");
        ledServo.setPosition(STEMperFiConstants.GB_LED_VIOLET);
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
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //odo.recalibrateIMU();
        odo.resetPosAndIMU();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        odo.resetPosAndIMU();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
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

        if (triangle2ButtonReader.wasJustPressed()) {
            attackStartTime = 0;
            pincherPosition = STEMperFiConstants.PINCHER_OPEN;
            armServoPosition = STEMperFiConstants.ARM_AIM;
            ledServo.setPosition(STEMperFiConstants.GB_LED_RED);
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
            ledServo.setPosition(STEMperFiConstants.GB_LED_GREEN);
        } else if (square2ButtonReader.wasJustPressed()) {
            attackStartTime = 0;
            armServoPosition = STEMperFiConstants.ARM_DRIVE;
            ledServo.setPosition(STEMperFiConstants.GB_LED_ORANGE);
        }
        armServo.setPosition(armServoPosition);
        if (attackStartTime > 0) {
            long attackDuration = System.currentTimeMillis() - attackStartTime;
            if (attackDuration > STEMperFiConstants.ATTACK_DRIVE_ms) {
                attackStartTime = 0;
                armServoPosition = STEMperFiConstants.ARM_DRIVE;
                ledServo.setPosition(STEMperFiConstants.GB_LED_ORANGE);
                armServo.setPosition(armServoPosition);
            } else if ( attackDuration > STEMperFiConstants.ATTACK_PINCHER_CLOSE_ms ) {
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


        double leftTrigger = gpB.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double rightTrigger = gpB.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        if (rightTrigger > 0.1 || leftTrigger > 0.1) {
            bucketServo.setPosition(STEMperFiConstants.BUCKET_SCORE);
        } else {
            bucketServo.setPosition(STEMperFiConstants.BUCKET_INTAKE);
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
            armServoPosition = STEMperFiConstants.ARM_INIT;
            armServo.setPosition(armServoPosition);
            vMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (bucketVerticalPosition == STEMperFiConstants.SCORE_BUCKET_FIRST) {
                bucketVerticalPosition = STEMperFiConstants.SCORE_BUCKET_SECOND;
            } else {
                liftStartTime = System.currentTimeMillis();
                bucketVerticalPosition = STEMperFiConstants.SCORE_BUCKET_FIRST;
            }
        } else if (Math.abs(gp2LY) > 0.1 && Math.abs(vTargetDiff) < 100) {
            vMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bucketVerticalPosition += Math.round(gp2LY * 100);
            bucketVerticalPosition = Math.max(-50, bucketVerticalPosition);
        }
        if (System.currentTimeMillis() - liftStartTime > STEMperFiConstants.LIFT_DELAY_MS) {
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
//        telemetry.addData("HR:", odo.getHeading());
//        telemetry.addData("Heading: ", degrees);


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

}
