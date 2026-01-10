package frc.robot.subsystems.localization;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

public class LocalizationConstants {
    public static final double ROTATION_SPEED_UPDATE_LIMIT = 0.058;

    public static final double MINIMUM_INTERPOLATION_DISTANCE = 0.3;
    public static final double MAXIMUM_INTERPOLATION_DISTANCE = 3.5;

    public static final double BEST_CASE_DEVS = 0.15;

    public static final double WORST_CASE_DEVS = 1;

    public static final double MULTI_TAG_MIN_DEVS = 0.15;
    public static final double MULTI_TAG_MAX_DEVS = 0.7;

    public static final double SINGLE_TAG_MIN_DEVS = 0.15;
    public static final double SINGLE_TAG_MAX_DEVS = 1;

    public static final double VISION_WEIGHT_YAW = 999999999;

    public static final double MAX_AUTO_UPDATE_DISTANCE = 6.5;


    public static final Vector<N3> FULL_CONTROL_WEIGHT = VecBuilder.fill(0.00000000001, 0.00000000001, 0.00000000000001);

    public static final int TIMES_TO_UPDATE = 25;

    public static final double SKID_DEBOUNCE_TIME = 3;

    public static final double ROTATE_ROBOT = 180;

    public static final double ROBOT_YAW_THRESHOLD = 2;

    public static final double CLOSEST_INTERPOLATED_DISTANCE = 0.5;
    public static final double FURTHEST_INTERPOLATED_DISTANCE = 3;
    public static final double FURTHEST_MULTIPLIER = 1;
    public static final double CLOSEST_MULTIPLIER = 1;
}
