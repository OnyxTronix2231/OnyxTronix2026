package frc.robot.data;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public class FieldConstants {

    public static final double FIELD_MIN_X = 0;
    public static final double FIELD_MIN_Y = 0;

    public static final double FIELD_MAX_X = 16.540988;
    public static final double FIELD_MAX_Y = 8.069326;

    public static Pose2d flipPose(Pose2d pose) {
        return new Pose2d(FIELD_MAX_X - pose.getX(), FIELD_MAX_Y - pose.getY(), pose.getRotation().rotateBy(Rotation2d.kPi));
    }

    public static boolean isInField(Pose2d pose, double paddingMeters) {
        if (pose.getX() <= FIELD_MIN_X + paddingMeters)
            return false;

        if (pose.getY() <= FIELD_MIN_Y + paddingMeters)
            return false;

        if (pose.getX() > FIELD_MAX_X + paddingMeters)
            return false;

        if (pose.getY() > FIELD_MAX_Y + paddingMeters)
            return false;
        return true;
    }

    public static Pose2d BLUE_RIGHT_BUMP  = new Pose2d(4.625594,2.51079,new Rotation2d(Math.toRadians(0)));

    public static Pose2d BLUE_LEFT_BUMP  = new Pose2d(4.625594,5.5523892,new Rotation2d(Math.toRadians(0)));

    public static Pose2d BLUE_RIGHT_TRANCHE = new Pose2d(4.625594,0.644652,new Rotation2d(Math.toRadians(0)));

    public static Pose2d BLUE_LEFT_TRANCHE = new Pose2d(4.625594,7.4185272,new Rotation2d(Math.toRadians(0)));

    public static Pose3d BLUE_HUB = new Pose3d(4.625594,4.034536, 1.8288, new Rotation3d(0, 0, 0));
}
