package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Qual New", group="FTC Lib")
@Disabled
public class QualifierNew extends OpMode
{
    // Create an instance of the sensor
    SparkFunOTOS myOtos;

    Gamepad.RumbleEffect customRumbleEffect;    // Use to build a custom rumble sequence.

    double oldTime = 0;
    int bucketVerticalPosition = 0;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor hMotor, vMotor, wMotor, llMotor, lrMotor;
    // input motors exactly as shown below
    MecanumDrive mecanum;
    GamepadEx gpA, gpB;

    long attackStartTime =  0;
    long liftStartTime = 0;
    long odoResetTime = 0;
    long climbStartTime = 0;
    long climbTargetPosition = 0;

    Servo bucketServo, ledServo, armServo, pincherServo, linkServo;
    double pincherPosition = STEMperFiConstants.PINCHER_CLOSE;
    double linkServoPosition = STEMperFiConstants.LINK_INIT;
    boolean iSpecimenMode = false;
    boolean isClimbMode = false;

    ButtonReader square2ButtonReader, triangle2ButtonReader, circle2ButtonReader, x2ButtonReader, dUp2ButtonReader, dDown2ButtonReader, dLeft2ButtonReader, dRight2ButtonReader, leftStick2ButtonReader, rightStick2ButtonReader, bumpLeft2ButtonReader, bumpRight2ButtonReader;

    double armServoPosition = STEMperFiConstants.NEW_ARM_INIT;
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

        bumpLeft2ButtonReader = new ButtonReader(gpB, GamepadKeys.Button.LEFT_BUMPER);
        bumpRight2ButtonReader = new ButtonReader(gpB, GamepadKeys.Button.RIGHT_BUMPER);

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

        linkServo = hardwareMap.get(Servo.class, "link");
        linkServo.setPosition(linkServoPosition);
//        hMotor = hardwareMap.get(DcMotor.class, "h");
//        hMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        hMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        hMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        vMotor = hardwareMap.get(DcMotor.class, "v");
        vMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        vMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vMotor.setTargetPosition(0);
        vMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vMotor.setPower(1);

        wMotor = hardwareMap.get(DcMotor.class, "w");
        wMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        llMotor = hardwareMap.get(DcMotor.class, "ll");
        llMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        llMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        llMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        llMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lrMotor = hardwareMap.get(DcMotor.class, "lr");
        lrMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lrMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        myOtos = myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        // All the configuration for the OTOS is done in this helper method, check it out!
        configureOtos();

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        myOtos.resetTracking();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        linkServoPosition = STEMperFiConstants.LINK_SCORE;
        linkServo.setPosition(linkServoPosition);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        double lx = gpA.getLeftX();
        double ly = gpA.getLeftY();
        double rx = gpA.getRightX();
        if (gamepad1.left_bumper || vMotor.getCurrentPosition() > 400) {
            lx = lx / 2;
            ly = ly / 2;
            rx = rx / 2;
        }
        double degrees =pos.h;

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
                myOtos.resetTracking();
                odoResetTime = now;
                gamepad1.runRumbleEffect(customRumbleEffect);

                ledServo.setPosition(STEMperFiConstants.GB_LED_YELLOW);
            }
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
        bumpLeft2ButtonReader.readValue();
        bumpRight2ButtonReader.readValue();

        if (!isClimbMode) {
            if (bumpLeft2ButtonReader.wasJustPressed() && bumpRight2ButtonReader.isDown() || bumpRight2ButtonReader.wasJustPressed() && bumpLeft2ButtonReader.isDown()) {
                // start lift mode timer
                climbStartTime = System.currentTimeMillis();
            }

            if (bumpLeft2ButtonReader.isDown() && bumpRight2ButtonReader.isDown()) {
                if (climbStartTime > 0 && System.currentTimeMillis() - climbStartTime > STEMperFiConstants.CLIMB_MODE_ACTIVATION_MS) {
                    isClimbMode = true;
                    gamepad2.runRumbleEffect(customRumbleEffect);
                    // TODO return everything to init position
                }
            } else {
                climbStartTime = 0;
            }

        if (dRight2ButtonReader.wasJustPressed() && !isClimbMode) {
            this.iSpecimenMode = !this.iSpecimenMode;
            if (this.iSpecimenMode) {
                pincherPosition = STEMperFiConstants.PINCHER_CLOSE;
                armServoPosition = STEMperFiConstants.NEW_ARM_INIT;
                bucketServo.setPosition(STEMperFiConstants.NEW_BUCKET_INIT);
                ledServo.setPosition(STEMperFiConstants.GB_LED_INDIGO);
                linkServoPosition = STEMperFiConstants.LINK_SCORE;
            } else {
                ledServo.setPosition(STEMperFiConstants.GB_LED_OFF);
            }
        }

        if (iSpecimenMode) {
            if (triangle2ButtonReader.wasJustPressed()) {
                attackStartTime = 0;
                pincherPosition = STEMperFiConstants.PINCHER_OPEN;
                armServoPosition = STEMperFiConstants.NEW_ARM_AIM;
                ledServo.setPosition(STEMperFiConstants.GB_LED_RED);
            } else if (x2ButtonReader.wasJustPressed() || dLeft2ButtonReader.wasJustPressed()) {
                attackStartTime = 0;
                armServoPosition = STEMperFiConstants.NEW_ARM_DRIVE;
                linkServoPosition = STEMperFiConstants.LINK_SCORE;
            } else if (circle2ButtonReader.wasJustPressed()) {
                attackStartTime = System.currentTimeMillis();
                armServoPosition = STEMperFiConstants.NEW_ARM_INTAKE;
                ledServo.setPosition(STEMperFiConstants.GB_LED_GREEN);
            } else if (square2ButtonReader.wasJustPressed()) {
                attackStartTime = 0;
                armServoPosition = STEMperFiConstants.NEW_ARM_DRIVE;
                ledServo.setPosition(STEMperFiConstants.GB_LED_ORANGE);
            }
        } else {
            if (triangle2ButtonReader.wasJustPressed()) {
                attackStartTime = 0;
                pincherPosition = STEMperFiConstants.PINCHER_OPEN;
                armServoPosition = STEMperFiConstants.NEW_ARM_AIM;
                ledServo.setPosition(STEMperFiConstants.GB_LED_RED);
            } else if (x2ButtonReader.wasJustPressed()) {
                attackStartTime = 0;
                armServoPosition = STEMperFiConstants.NEW_ARM_SCORE;
                linkServoPosition = STEMperFiConstants.LINK_SCORE;
            } else if (circle2ButtonReader.wasJustPressed()) {
                attackStartTime = System.currentTimeMillis();
                armServoPosition = STEMperFiConstants.NEW_ARM_INTAKE;
                ledServo.setPosition(STEMperFiConstants.GB_LED_GREEN);
            } else if (square2ButtonReader.wasJustPressed()) {
                attackStartTime = 0;
                armServoPosition = STEMperFiConstants.NEW_ARM_DRIVE;
                ledServo.setPosition(STEMperFiConstants.GB_LED_ORANGE);
            }
        }
        armServo.setPosition(armServoPosition);
        if (attackStartTime > 0) {
            long attackDuration = System.currentTimeMillis() - attackStartTime;
            if (attackDuration > STEMperFiConstants.ATTACK_DRIVE_ms) {
                attackStartTime = 0;
                armServoPosition = STEMperFiConstants.NEW_ARM_DRIVE;
                ledServo.setPosition(STEMperFiConstants.GB_LED_ORANGE);
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
                bucketServo.setPosition(STEMperFiConstants.NEW_BUCKET_SCORE);
            } else {
                bucketServo.setPosition(STEMperFiConstants.NEW_BUCKET_INTAKE);
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
            armServoPosition = iSpecimenMode ? STEMperFiConstants.NEW_ARM_INIT : STEMperFiConstants.NEW_ARM_INIT_SAMPLE;
            armServo.setPosition(armServoPosition);
            vMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (bucketVerticalPosition == STEMperFiConstants.NEW_SCORE_BUCKET_FIRST) {
                bucketVerticalPosition = STEMperFiConstants.NEW_SCORE_BUCKET_SECOND;
            } else {
                liftStartTime = System.currentTimeMillis();
                bucketVerticalPosition = iSpecimenMode ? STEMperFiConstants.NEW_SCORE_BUCKET_SPECIMEN : STEMperFiConstants.NEW_SCORE_BUCKET_FIRST;
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
        telemetry.addData("pos x", pos.x);
        telemetry.addData("pos y", pos.y);
        telemetry.addData("pos h", pos.h);



        double gp2RY = gpB.getRightY();
        if (dRight2ButtonReader.wasJustPressed()) {
              linkServoPosition = STEMperFiConstants.LINK_SCORE;
        } else if (Math.abs(gp2RY) > 0.1) {
            linkServoPosition += gp2RY * STEMperFiConstants.LINK_INCREMENT;
        }
        linkServoPosition = Math.min(linkServoPosition, STEMperFiConstants.LINK_INIT);
        linkServoPosition = Math.max(linkServoPosition, STEMperFiConstants.LINK_OUT);
        linkServo.setPosition(linkServoPosition);

        telemetry.addData("v", gp2LY);
        telemetry.addData("targetPosition", bucketVerticalPosition);
        telemetry.addData("vMotor.getTargetPosition)", vMotor.getTargetPosition());
        telemetry.addData("vMotor.getCurrentPosition()", vMotor.getCurrentPosition());
        } else {
            // climb mode
            incrementRainbow();
            double winchPower = gpB.getLeftY();
            double liftPower = -gpB.getRightY();

            winchPower = Math.abs(winchPower) > 0.1 ? winchPower : 0;
            wMotor.setPower(winchPower);

            liftPower = Math.abs(liftPower) > 0.1 ? liftPower : 0;
            if (liftPower != 0) {
                int newTarget = (int) (llMotor.getCurrentPosition() + (20 * liftPower));
                llMotor.setTargetPosition(newTarget);
                lrMotor.setTargetPosition(newTarget);
                lrMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                llMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                llMotor.setPower(1);
                lrMotor.setPower(1);
            }

            telemetry.addData("winch Power:", winchPower);
            telemetry.addData("lift Power: ", liftPower);
            telemetry.addData("targetPos: ", llMotor.getTargetPosition());
            telemetry.addData("curPos: ", llMotor.getCurrentPosition());
        }


//        telemetry.addData("h", gp2RY);
//        telemetry.addData("hMotor.getCurrentPosition()", hMotor.getCurrentPosition());
//        telemetry.addData("hMotor.getTargetPosition)", hMotor.getTargetPosition());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void incrementRainbow() {
        double newVal = ledServo.getPosition() + STEMperFiConstants.RAINBOW_INCREMENT;
        if (newVal > STEMperFiConstants.GB_LED_VIOLET || newVal < STEMperFiConstants.GB_LED_RED) {
            newVal = STEMperFiConstants.GB_LED_RED;
        }
        ledServo.setPosition(newVal);
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
