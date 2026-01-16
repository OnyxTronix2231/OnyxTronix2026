package frc.robot.data;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import static frc.robot.data.FieldConstants.*;

public class FieldUtils {

    public static Pose2d flipPose(Pose2d pose) {
        return new Pose2d(FIELD_MAX_X - pose.getX(), FIELD_MAX_Y - pose.getY(), pose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
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
}
