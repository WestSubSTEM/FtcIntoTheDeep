package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public class SparkFunOTOSOdometry extends Odometry {

    private final SparkFunOTOS otos;

    public SparkFunOTOSOdometry(SparkFunOTOS otos, double trackwidth) {
        super(sparkFunPose2ftcLibPose(otos.getPosition()), trackwidth);
        this.otos = otos;
    }

    private static Pose2d sparkFunPose2ftcLibPose(SparkFunOTOS.Pose2D sarkfunPose2D) {
        return new Pose2d(sarkfunPose2D.x, sarkfunPose2D.y, Rotation2d.fromDegrees(sarkfunPose2D.h));
    }

    private static SparkFunOTOS.Pose2D ftcLibPose2sparkFunPose(Pose2d pose2d) {
        return new SparkFunOTOS.Pose2D(pose2d.getX(), pose2d.getY(), Math.toRadians(pose2d.getHeading()));
    }

    /**
     * This handles all the calculations for you.
     */
    @Override
    public void updatePose() {
        robotPose = sparkFunPose2ftcLibPose(otos.getPosition());
    }

    @Override
    public void updatePose(Pose2d pose) {
        otos.setPosition(ftcLibPose2sparkFunPose(pose));
        robotPose = pose;
    }

}
