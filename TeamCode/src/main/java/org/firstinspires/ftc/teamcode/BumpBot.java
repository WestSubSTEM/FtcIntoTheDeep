/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.sparkfun.SparkFunLEDStick;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Drive", group="FTC Lib")
@Disabled
public class BumpBot extends OpMode
{
    //GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    double oldTime = 0;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
//    MotorEx hMotor, vMotor;
    // input motors exactly as shown below
    MecanumDrive mecanum;
    GamepadEx gpA, gpB;
    GamepadButton leftBumper, rightBumper;
    //SparkFunLEDStick led;
    //RevIMU imu;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
//        imu = new RevIMU(hardwareMap);
//        imu.init();
//
//        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
//        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
//
//        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);


        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        //imu.initialize(new IMU.Parameters(orientationOnRobot));


        // the extended gamepad object
        gpA = new GamepadEx(gamepad1);
        gpB = new GamepadEx(gamepad2);

//        leftBumper = new GamepadButton(
//                driverOp, GamepadKeys.Button.LEFT_BUMPER);
//        rightBumper = new GamepadButton(driverOp, GamepadKeys.Button.RIGHT_BUMPER);

        Motor frontLeft = new Motor(hardwareMap, "fl", Motor.GoBILDA.RPM_312);
        Motor frontRight = new Motor(hardwareMap, "fr", Motor.GoBILDA.RPM_312);
        Motor backLeft = new Motor(hardwareMap, "bl", Motor.GoBILDA.RPM_312);
        Motor backRight = new Motor(hardwareMap, "br", Motor.GoBILDA.RPM_312);


       // mecanum = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        mecanum = new MecanumDrive(backRight, backLeft , frontRight, frontLeft);

//        hMotor = new MotorEx(hardwareMap, "h", Motor.GoBILDA.RPM_117);
//        hMotor.setInverted(true);
//        hMotor.encoder.reset();
//        hMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//
//        vMotor = new MotorEx(hardwareMap, "v", Motor.GoBILDA.RPM_60);
//        vMotor.setInverted(true);
//        vMotor.encoder.reset();
//        vMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

//        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

      //  led = hardwareMap.get(SparkFunLEDStick.class, "led");

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
  //      odo.setOffsets(-84.0, -168.0); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
      //  odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        //odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);


        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //odo.recalibrateIMU();
        //odo.resetPosAndIMU();


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
       // odo.resetPosAndIMU();

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
        //odo.update();
        double lx = gpA.getLeftX();
        double ly = gpA.getLeftY();
        double rx = gpA.getRightX();
//        double degrees = Math.toDegrees(odo.getHeading());
//        double degrees = imu.getRotation2d().getDegrees();
//

            telemetry.addLine("Robot Centric");
            mecanum.driveRobotCentric(
                    lx,
                    ly,
                    -rx,
                    false
            );
//        } else {
//            telemetry.addLine("Field Centric");
//            mecanum.driveFieldCentric(
//                    lx,
//                    ly,
//                    rx,
//                    degrees,   // gyro value passed in here must be in degrees
//                    false
//            );
//        }
        //telemetry.addData("degrees", degrees);
        telemetry.addData("Left X", lx);
        telemetry.addData("Left Y", ly);
        telemetry.addData("Right X", rx);
        telemetry.addData("Raw X", gamepad1.right_stick_x);

       // telemetry.addData("HR:", odo.getHeading());
      //  telemetry.addData("Heading: ", degrees);
//        double gp2LY = gpB.getLeftY();
//        double gp2RX = gpB.getRightX();

//        telemetry.addData("v", gp2LY);
//        telemetry.addData("vMotor.encoder.getPosition()", vMotor.encoder.getPosition());
//        telemetry.addData("vMotor.encoder.getDistance()", vMotor.encoder.getDistance());
//        vMotor.set(gp2LY);
//        telemetry.addData("h", gp2RX);
//        telemetry.addData("hMotor.getCurrentPosition()", hMotor.getCurrentPosition());
//        telemetry.addData("hMotor.getDistance()", hMotor.getDistance());
//        telemetry.addData("hMotor.encoder.getPosition()", hMotor.encoder.getPosition());
//        telemetry.addData("hMotor.getDistance()", hMotor.encoder.getDistance());
//        hMotor.set(gp2RX);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
