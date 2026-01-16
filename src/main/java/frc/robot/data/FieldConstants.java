package frc.robot.data;

import edu.wpi.first.math.geometry.*;

public class FieldConstants {

    public static final double FIELD_MIN_X = 0;
    public static final double FIELD_MIN_Y = 0;

    public static final double FIELD_MAX_X = 16.540988;
    public static final double FIELD_MAX_Y = 8.069326;

    public static Pose2d BLUE_RIGHT_BUMP = new Pose2d(4.625594, 2.51079, new Rotation2d(Math.toRadians(0)));
    public static Pose2d BLUE_LEFT_BUMP = new Pose2d(4.625594, 5.5523892, new Rotation2d(Math.toRadians(0)));
    public static double BUMP_LENGTH_Y = 1.854;
    public static double BUMP_LENGTH_X = 1.128+0.2;
    public static double TRENCH_LENGTH_Y = 1.284986;
    public static double TRENCH_LENGTH_X = 1.128+0.2;

    public static Pose2d BLUE_RIGHT_TRENCH = new Pose2d(4.625594, 0.644652, new Rotation2d(Math.toRadians(0)));
    public static Pose2d BLUE_LEFT_TRENCH = new Pose2d(4.625594, 7.4185272, new Rotation2d(Math.toRadians(0)));

    public static Pose2d BLUE_RIGHT_TOWER = new Pose2d(1.06172, 3.730244-0.5969, new Rotation2d(Math.toRadians(0)));
    public static Pose2d BLUE_LEFT_TOWER = new Pose2d(1.06172, 3.730244+0.5969, new Rotation2d(Math.toRadians(0)));

    public static Pose2d BLUE_DEPOT = new Pose2d(0.3937, 5.94995, new Rotation2d(0));

    public static Pose2d BLUE_FEEDER = new Pose2d(0, 0.650748, new Rotation2d(0));

    public static Pose2d BLUE_RIGHT_DELIVERY = new Pose2d(1.8446496, 1.5, new Rotation2d(Math.toRadians(0)));
    public static Pose2d BLUE_LEFT_DELIVERY = new Pose2d(1.8446496, FIELD_MAX_Y - 1.5, new Rotation2d(Math.toRadians(0)));

    public static Pose3d BLUE_HUB = new Pose3d(4.625594, 4.034536, 1.8288, new Rotation3d(0, 0, 0));
    public static double HUB_LENGTH = 1.1938;

    public static Translation2d BLUE_HUB_TOP_RIGHT = new Translation2d(4.625594+HUB_LENGTH/2, 4.034536+HUB_LENGTH/2);
    public static Translation2d BLUE_HUB_TOP_LEFT = new Translation2d(4.625594+HUB_LENGTH/2, 4.034536-HUB_LENGTH/2);
    public static Translation2d BLUE_HUB_BOTTOM_RIGHT = new Translation2d(4.625594-HUB_LENGTH/2, 4.034536+HUB_LENGTH/2);
    public static Translation2d BLUE_HUB_BOTTOM_LEFT = new Translation2d(4.625594-HUB_LENGTH/2, 4.034536-HUB_LENGTH/2);

    public static double CLIMBING_DISTANCE = 0.5;
}
