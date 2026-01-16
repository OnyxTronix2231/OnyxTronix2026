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

    public static Pose2d BLUE_RIGHT_BUMP = new Pose2d(4.625594, 2.51079, new Rotation2d(Math.toRadians(0)));

    public static Pose2d BLUE_LEFT_BUMP = new Pose2d(4.625594, 5.5523892, new Rotation2d(Math.toRadians(0)));

    public static Pose2d BLUE_RIGHT_TRENCH = new Pose2d(4.625594, 0.644652, new Rotation2d(Math.toRadians(0)));

    public static Pose2d BLUE_LEFT_TRENCH = new Pose2d(4.625594, 7.4185272, new Rotation2d(Math.toRadians(0)));

    public static Pose2d BLUE_TOWER = new Pose2d(1.06172, 3.730244, new Rotation2d(Math.toRadians(0)));

    public static Pose2d BLUE_DEPOT = new Pose2d(0.3937, 5.94995, new Rotation2d(0));

    public static Pose2d BLUE_FEEDER = new Pose2d(0, 0.650748, new Rotation2d(0));

    public static Pose2d BLUE_RIGHT_DELIVERY = new Pose2d(1.8446496, 1.5, new Rotation2d(Math.toRadians(0)));

    public static Pose2d BLUE_LEFT_DELIVERY = new Pose2d(1.8446496, FIELD_MAX_Y - 1.5, new Rotation2d(Math.toRadians(0)));

    public static Pose3d BLUE_HUB = new Pose3d(4.625594, 4.034536, 1.8288, new Rotation3d(0, 0, 0));
    public static double HUB_LENGTH = 1.1938;
}
