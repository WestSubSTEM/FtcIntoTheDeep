package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@Autonomous(name="Pure Pursuit Test", group="FTC Lib")
public class PurePursuitSample extends CommandOpMode {
    //
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    // define our constants
    static final double TRACKWIDTH = 13.7;
    static final double WHEEL_DIAMETER = 4.0;    // inches
    static double TICKS_TO_INCHES;
    static final double CENTER_WHEEL_OFFSET = 2.4;

    private GoBildaPinpointOdometry m_robotOdometry;
    private OdometrySubsystem m_odometry;
    private PurePursuitCommand ppCommand;
    private MecanumDrive m_robotDrive;
    private Motor fL, fR, bL, bR;
    private MotorEx leftEncoder, rightEncoder, centerEncoder;

    @Override
    public void initialize() {
        Motor fL = new Motor(hardwareMap, "fl", Motor.GoBILDA.RPM_312);
        Motor fR = new Motor(hardwareMap, "fr", Motor.GoBILDA.RPM_312);
        Motor bL = new Motor(hardwareMap, "bl", Motor.GoBILDA.RPM_312);
        Motor bR = new Motor(hardwareMap, "br", Motor.GoBILDA.RPM_312);

        // create our drive object
        m_robotDrive = new MecanumDrive(fL, fR, bL, bR);


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

        // create our odometry object and subsystem
        m_robotOdometry = new GoBildaPinpointOdometry(odo, TRACKWIDTH);
        m_odometry = new OdometrySubsystem(m_robotOdometry);

        // create our pure pursuit command
        ppCommand = new PurePursuitCommand(
                m_robotDrive, m_odometry,
                new StartWaypoint(0, 10),
                new GeneralWaypoint(0, 20, .2, 0.2, 30),
                new EndWaypoint(
                        0, 30, 0, 0.2,
                        0.5, 30, 0.8, 1
                )
        );

        // schedule the command
        schedule(ppCommand);
    }

}
