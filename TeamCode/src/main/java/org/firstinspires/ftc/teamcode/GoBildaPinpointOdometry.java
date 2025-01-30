package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.Odometry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class GoBildaPinpointOdometry extends Odometry {

    private final GoBildaPinpointDriver pinpoint;

    public GoBildaPinpointOdometry(GoBildaPinpointDriver pinpoint, double trackwidth) {
        super(new Pose2d(), trackwidth);
        this.pinpoint = pinpoint;
    }

    /**
     * This handles all the calculations for you.
     */
    @Override
    public void updatePose() {
        pinpoint.update();
        robotPose = new Pose2d(pinpoint.getPosX(), pinpoint.getPosY(), new Rotation2d(pinpoint.getHeading()));
    }

    @Override
    public void updatePose(Pose2d pose) {
        org.firstinspires.ftc.robotcore.external.navigation.Pose2D p2 = new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading());
        pinpoint.setPosition(p2);
        robotPose = pose;
    }

}
