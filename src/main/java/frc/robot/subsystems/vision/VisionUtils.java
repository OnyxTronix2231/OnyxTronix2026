package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

import static frc.robot.subsystems.vision.VisionConstants.*;

public class VisionUtils {
    public static double interpolate(double distance, double minDev, double maxDev, double minDist, double maxDist) {
        distance = edu.wpi.first.math.MathUtil.clamp(distance, minDist, maxDist);
        double rawVal = edu.wpi.first.math.MathUtil.inverseInterpolate(minDist, maxDist, distance);

        return edu.wpi.first.math.MathUtil.interpolate(minDev, maxDev, rawVal);
    }

    public static double calculateTriangleArea(Point a, Point b, Point c) {
        return Math.abs((a.x() * b.y() - c.y() + b.x() * (c.y() - a.y()) +
                c.x() * (a.y() - b.y())) / 2.0);
    }


    public static boolean isintriangle(Pose2d botPose) {
        Point pose = new Point(botPose.getX(), botPose.getY());

        return (
                (isAboveSlope(pose, RED_RIGHT_CORNER_C, RED_RIGHT_CORNER_A) ||
                        isAboveSlope(pose, RED_LEFT_CORNER_C, RED_LEFT_CORNER_A) ||
                        isAboveSlope(pose, BLUE_LEFT_CORNER_A, BLUE_LEFT_CORNER_C) ||
                        isAboveSlope(pose, BLUE_RIGHT_CORNER_B, BLUE_RIGHT_CORNER_A)
                ));


    }

    public static double calculateSlopeOfTwoPoses(Point pose1, Point pose2) {
        return (pose2.y() - pose1.y()) / (pose2.x() - pose1.x());
    }

    public static boolean isAboveSlope(Point botPose, Point pointA, Point pointB) {
        double poseSlope = calculateSlopeOfTwoPoses(pointA, pointB);
        if (Math.signum(calculateSlopeOfTwoPoses(pointA, botPose)) != Math.signum(poseSlope)) {
            return false;
        } else if (Math.abs(calculateSlopeOfTwoPoses(pointA, botPose)) < Math.abs(poseSlope)) {
            return false;
        }
        return true;
    }

    public static boolean isInField(Pose2d pose, double paddingMeters) {
        if (pose.getX() <= FIELD_MIN_X - paddingMeters)
            return false;

        if (pose.getY() <= FIELD_MIN_Y - paddingMeters)
            return false;

        if (pose.getX() > FIELD_MAX_X + paddingMeters)
            return false;

        if (pose.getY() > FIELD_MAX_Y + paddingMeters)
            return false;

        if (isintriangle(pose))
            return false;

        return !isintriangle(pose);
    }
}