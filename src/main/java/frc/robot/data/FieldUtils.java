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

    public static boolean isInRect(Pose2d pose, double x, double y, Pose2d center) {
        if (pose.getX() <= center.getX()-x/2)
            return false;

        if (pose.getY() <= center.getY()-y/2)
            return false;

        if (pose.getX() > center.getX()+x/2)
            return false;

        if (pose.getY() > center.getY()+y/2)
            return false;
        return true;
    }

    public static Pose2d nearestPos(Pose2d pos, Pose2d pos1, Pose2d pos2) {
        if (pos.getTranslation().getDistance(pos1.getTranslation()) < pos.getTranslation().getDistance(pos2.getTranslation()))
            return pos1;
        return pos2;
    }
}
