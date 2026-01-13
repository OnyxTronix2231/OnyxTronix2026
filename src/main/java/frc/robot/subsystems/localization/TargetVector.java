package frc.robot.subsystems.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;

import static frc.robot.subsystems.localization.LocalizationConstants.ROTATE_ROBOT;

public class TargetVector {
    public static Translation3d get3DRobotToFieldPointOffset(Translation3d fieldPoint) {
        Pose3d pose = new Pose3d(Localization.getInstance().getBotPose());
        Translation3d botTranslation = pose.getTranslation();

        return fieldPoint.minus(botTranslation);
    }

    public static double get2dDistanceFromFieldPoint(Translation3d fieldPoint) {
        Translation3d robotToTargetVector = get3DRobotToFieldPointOffset(fieldPoint);
        return Math.hypot(robotToTargetVector.getX(), robotToTargetVector.getY());
    }

    public static double getYawInDegreesFromFieldPoint(Translation3d fieldPoint) {
        Translation3d robotToTargetVector = get3DRobotToFieldPointOffset(fieldPoint);
        return Math.toDegrees(Math.atan2(robotToTargetVector.getY(), robotToTargetVector.getX())) + ROTATE_ROBOT;
    }

    public static Pose2d getRelativeOffsetFromFieldPoint(Pose2d fieldPoint) {
        Pose2d botPose = Localization.getInstance().getBotPose();
        return botPose.relativeTo(fieldPoint);
    }
}
