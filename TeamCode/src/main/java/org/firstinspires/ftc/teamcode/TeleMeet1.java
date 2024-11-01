package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Tele Meet 1", group="FTC Lib")
public class TeleMeet1 extends OpMode
{
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    double oldTime = 0;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    MotorEx hMotor, vMotor;
    // input motors exactly as shown below
    MecanumDrive mecanum;
    GamepadEx gpA, gpB;
    GamepadButton leftBumper, rightBumper;

    Servo servoBucket, servoLED;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // the extended gamepad object
        gpA = new GamepadEx(gamepad1);
        gpB = new GamepadEx(gamepad2);

        servoBucket = hardwareMap.get(Servo.class, "bucket");
        servoBucket.setPosition(STEMperFiConstants.BUCKET_INIT);
        servoLED = hardwareMap.get(Servo.class, "gbled");
        servoLED.setPosition(STEMperFiConstants.GB_LED_VIOLET);

        Motor frontLeft = new Motor(hardwareMap, "fl", Motor.GoBILDA.RPM_312);
        Motor frontRight = new Motor(hardwareMap, "fr", Motor.GoBILDA.RPM_312);
        Motor backLeft = new Motor(hardwareMap, "bl", Motor.GoBILDA.RPM_312);
        Motor backRight = new Motor(hardwareMap, "br", Motor.GoBILDA.RPM_312);

        mecanum = new MecanumDrive(backRight, backLeft , frontRight, frontLeft);

        hMotor = new MotorEx(hardwareMap, "h", Motor.GoBILDA.RPM_117);
        hMotor.setInverted(true);
        hMotor.encoder.reset();
        hMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        vMotor = new MotorEx(hardwareMap, "v", Motor.GoBILDA.RPM_60);
        vMotor.setInverted(true);
        vMotor.encoder.reset();
        vMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

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
        double degrees = Math.toDegrees(odo.getHeading());

        mecanum.driveFieldCentric(
                lx,
                ly,
                rx,
                degrees,   // gyro value passed in here must be in degrees
                false
        );

        // Gamepad 2
        if (gamepad2.right_bumper || gamepad2.left_bumper) {
            servoBucket.setPosition(STEMperFiConstants.BUCKET_SCORE);
        } else {
            servoBucket.setPosition(STEMperFiConstants.BUCKET_INTAKE);
        }
        if (gamepad2.right_trigger > 0.1) {

        } else if (gamepad2.left_trigger > 0.1) {

        } else {

        }



//        telemetry.addData("Left X", lx);
//        telemetry.addData("Left Y", ly);
//        telemetry.addData("Right X", rx);
//        telemetry.addData("HR:", odo.getHeading());
//        telemetry.addData("Heading: ", degrees);

        double gp2LY = gpB.getLeftY();
        double gp2RX = gpB.getRightX();

        telemetry.addData("v", gp2LY);
        telemetry.addData("vMotor.encoder.getPosition()", vMotor.encoder.getPosition());
        telemetry.addData("vMotor.encoder.getDistance()", vMotor.encoder.getDistance());
        vMotor.set(gp2LY);
        telemetry.addData("h", gp2RX);
        telemetry.addData("hMotor.getCurrentPosition()", hMotor.getCurrentPosition());
        telemetry.addData("hMotor.getDistance()", hMotor.getDistance());
        telemetry.addData("hMotor.encoder.getPosition()", hMotor.encoder.getPosition());
        telemetry.addData("hMotor.getDistance()", hMotor.encoder.getDistance());
        hMotor.set(gp2RX);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
