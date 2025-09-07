package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name="Drive Only", group="Test")
public class DriveOnly extends OpMode
{
      // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    // input motors exactly as shown below
    MecanumDrive mecanum;
    GamepadEx gpA, gpB;

    Motor frontLeft;
    Motor frontRight;
    Motor backLeft;
    Motor backRight;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

            // the extended gamepad object
        gpA = new GamepadEx(gamepad1);
        gpB = new GamepadEx(gamepad2);

        frontLeft = new Motor(hardwareMap, "fl", Motor.GoBILDA.BARE);
        frontRight = new Motor(hardwareMap, "fr", Motor.GoBILDA.BARE);
        backLeft = new Motor(hardwareMap, "bl", Motor.GoBILDA.BARE);
        backRight = new Motor(hardwareMap, "br", Motor.GoBILDA.BARE);

        mecanum = new MecanumDrive(backRight, backLeft , frontRight, frontLeft);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        frontRight.set(gamepad1.triangle ? .2 : 0);
        backRight.set(gamepad1.cross ? .2 : 0);
        frontLeft.set(gamepad1.dpad_up ? .2 : 0);
        backLeft.set(gamepad1.dpad_down ? .2 :0);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        double lx = gpA.getLeftX();
        double ly = gpA.getLeftY();
        double rx = gpA.getRightX();
        if (!gamepad1.left_bumper) {
            lx = lx / 2;
            ly = ly / 2;
            rx = rx / 2;
        }

        mecanum.driveRobotCentric(
                lx,
                ly,
                rx
        );
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        mecanum.driveRobotCentric(0,0,0);
    }

}
