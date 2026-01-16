package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import java.util.HashMap;
import java.util.Map;

public class VisionConstants {

    public static final double FIELD_MIN_X = 0;
    public static final double FIELD_MIN_Y = 0;

    public static final double FIELD_MAX_X = 17.5482504;
    public static final double FIELD_MAX_Y = 8.0518;

    public static final double PADDING_METERS = 0.1;

    public static final Point BLUE_RIGHT_CORNER_A = new Point(1.75 + PADDING_METERS, 0);
    public static final Point BLUE_RIGHT_CORNER_B = new Point(0, 1.25 + PADDING_METERS);
    public static final Point BLUE_RIGHT_CORNER_C = new Point(0, 0);

    public static final Point BLUE_LEFT_CORNER_A = new Point(0, 6.75 - PADDING_METERS);
    public static final Point BLUE_LEFT_CORNER_B = new Point(0, 8);
    public static final Point BLUE_LEFT_CORNER_C = new Point(1.75 + PADDING_METERS, 8);

    public static final Point RED_RIGHT_CORNER_A = new Point(15.8 - PADDING_METERS, 8);
    public static final Point RED_RIGHT_CORNER_B = new Point(17.5, 8);
    public static final Point RED_RIGHT_CORNER_C = new Point(17.5, 6.75 - PADDING_METERS);

    public static final Point RED_LEFT_CORNER_A = new Point(15.8 - PADDING_METERS, 0);
    public static final Point RED_LEFT_CORNER_B = new Point(17.5, 0);
    public static final Point RED_LEFT_CORNER_C = new Point(17.5, 1.25 + PADDING_METERS);

    public static final String LEFT_LIMELIGHT_NAME = "limelight-l";
    public static final String RIGHT_LIMELIGHT_NAME = "limelight-r";

    public static final double GAME_PIECE_RADIUS = 0.175;
    public static final double GAME_PIECE_HEIGHT = 0.05;

    public static final double OBJECT_DETECTION_HORIZONTAL_RESOLUTION = 640;
    public static final double OBJECT_DETECTION_VERTICAL_RESOLUTION = 480;

    public static final double DEGREES_OF_LEEWAY = 1.7;

    public static final double CORRECTION_EXPONENTIAL_LEFT = 2; // needs to be calibrated per camera.
    public static final double CORRECTION_EXPONENTIAL_RIGHT = 1; // needs to be calibrated per camera.

    public static final double UPPER_CROPPED_CORRECTION_MAGIC_NUMBER = 1.5; // needs to be calibrated per camera.
    public static final double LOWER_CROPPED_CORRECTION_MAGIC_NUMBER = 9.55; // needs to be calibrated per camera.

    public static final int SMALLEST_INTEGER_BIGGER_THAN_ONE = 2;

    public static final double FULL_TRUST_STD_DEVIATION = 0.0001;

    public static final double IMPOSSIBLE_DISTANCE = -0.1;
    public static final Rotation2d UNBELIEVABLE_ROTATION = Rotation2d.fromRotations(-9999999);
    public static final Translation2d IMPOSSIBLE_PIECE_TRANSLATION = new Translation2d(IMPOSSIBLE_DISTANCE, UNBELIEVABLE_ROTATION);
    public static final double DISTANCE_FROM_CAMERA_TO_INTAKE = 0.11;
    public static final double DISTANCE_FROM_CAMERA_TO_ROBOT = 0.342;
    public static final double DISTANCE_FROM_INTAKE_TO_ROBOT = DISTANCE_FROM_CAMERA_TO_INTAKE + DISTANCE_FROM_CAMERA_TO_ROBOT;
    public static final Translation2d INTAKE_TO_ROBOT_VECTOR = new Translation2d(DISTANCE_FROM_INTAKE_TO_ROBOT, new Rotation2d());

    public static final double UNRELIABLE_PITCH_THRESHOLD_TOP = 11.4;
    public static final double UNRELIABLE_PITCH_THRESHOLD_BOTTOM = -17.7;

    public static final int T_HOR_2D_ARRAY_INDEX = 12;
    public static final double ACCEPTABLE_DISK_ERROR = 0.15;
    public static final int NO_UPDATE_DEVS = 9999999;

    public static double PIXEL_TO_RADIANS(double pixels, double fov) {
        return (Math.toRadians(fov) * pixels) / OBJECT_DETECTION_HORIZONTAL_RESOLUTION;
    }

    public static Transform2d TRANSLATION_TO_TRANSFORM(Translation2d translation) {
        return new Transform2d(translation, new Rotation2d());
    }

    public static final InterpolatingDoubleTreeMap CROPPED_DISK_TO_DISTANCE = new InterpolatingDoubleTreeMap();

    public static void initializeMap() {
        CROPPED_DISK_TO_DISTANCE.put(0.282, 2.411347518);
        CROPPED_DISK_TO_DISTANCE.put(0.287, 2.996515679);
        CROPPED_DISK_TO_DISTANCE.put(0.29, 3.517241379);
        CROPPED_DISK_TO_DISTANCE.put(0.296, 4.324324324);
        CROPPED_DISK_TO_DISTANCE.put(0.299, 4.782608696);
        CROPPED_DISK_TO_DISTANCE.put(0.301, 5.282392027);
        CROPPED_DISK_TO_DISTANCE.put(0.305, 6.295081967);
        CROPPED_DISK_TO_DISTANCE.put(0.307, 7.198697068);
        CROPPED_DISK_TO_DISTANCE.put(0.31, 8.129032258);
        CROPPED_DISK_TO_DISTANCE.put(0.313, 8.75399361);
        CROPPED_DISK_TO_DISTANCE.put(0.315, 9.365079365);
        CROPPED_DISK_TO_DISTANCE.put(0.316, 11.13924051);
    }

    public static double correctCropOutput(double distance) {
        return CROPPED_DISK_TO_DISTANCE.get(distance) * distance;
    }

    public static final Map<Integer, Pose3d> APRIL_TAG_MAP = new HashMap<>() {
        {
            put(1,  new Pose3d(new Translation3d(11.8781, 7.4247, 0.8890),  new Rotation3d(0, 0, 180)));
            put(2,  new Pose3d(new Translation3d(11.9263, 4.6466, 1.1240),  new Rotation3d(0, 0, 90)));
            put(3,  new Pose3d(new Translation3d(11.3125, 4.5182, 1.1240),  new Rotation3d(0, 0, 180)));
            put(4,  new Pose3d(new Translation3d(11.3125, 4.0343, 1.1240),  new Rotation3d(0, 0, 180)));
            put(5,  new Pose3d(new Translation3d(11.9263, 3.4293, 1.1240),  new Rotation3d(0, 0, 270)));
            put(6,  new Pose3d(new Translation3d(11.8781, 0.6444, 0.8890),  new Rotation3d(0, 0, 180)));
            put(7,  new Pose3d(new Translation3d(11.9530, 0.6444, 0.8890),  new Rotation3d(0, 0, 0)));
            put(8,  new Pose3d(new Translation3d(12.2740, 3.4293, 1.1240),  new Rotation3d(0, 0, 270)));
            put(9,  new Pose3d(new Translation3d(12.5471, 3.6755, 1.1240),  new Rotation3d(0, 0, 0)));
            put(10, new Pose3d(new Translation3d(12.5471, 4.0343, 1.1240),  new Rotation3d(0, 0, 0)));
            put(11, new Pose3d(new Translation3d(12.2740, 4.6466, 1.1240),  new Rotation3d(0, 0, 90)));
            put(12, new Pose3d(new Translation3d(11.9530, 7.4247, 0.8890),  new Rotation3d(0, 0, 0)));
            put(13, new Pose3d(new Translation3d(16.5235, 7.4554, 0.5525),  new Rotation3d(0, 0, 180)));
            put(14, new Pose3d(new Translation3d(16.5235, 6.9701, 0.5525),  new Rotation3d(0, 0, 180)));
            put(15, new Pose3d(new Translation3d(16.5329, 4.3236, 0.5525),  new Rotation3d(0, 0, 180)));
            put(16, new Pose3d(new Translation3d(16.5329, 3.8903, 0.5525),  new Rotation3d(0, 0, 180)));
            put(17, new Pose3d(new Translation3d(4.6614, 0.6444, 0.8890),   new Rotation3d(0, 0, 0)));
            put(18, new Pose3d(new Translation3d(4.6256, 3.4293, 1.1240),  new Rotation3d(0, 0, 270)));
            put(19, new Pose3d(new Translation3d(5.2392, 3.6755, 1.1240),  new Rotation3d(0, 0, 0)));
            put(20, new Pose3d(new Translation3d(5.2392, 4.0343, 1.1240),  new Rotation3d(0, 0, 0)));
            put(21, new Pose3d(new Translation3d(4.6256, 4.6466, 1.1240),  new Rotation3d(0, 0, 90)));
            put(22, new Pose3d(new Translation3d(4.6614, 7.4247, 0.8890),  new Rotation3d(0, 0, 0)));
            put(23, new Pose3d(new Translation3d(4.5883, 7.4247, 0.8890),  new Rotation3d(0, 0, 180)));
            put(24, new Pose3d(new Translation3d(4.2824, 4.6466, 1.1240),  new Rotation3d(0, 0, 90)));
            put(25, new Pose3d(new Translation3d(4.0213, 4.5182, 1.1240),  new Rotation3d(0, 0, 180)));
            put(26, new Pose3d(new Translation3d(4.0213, 4.0343, 1.1240),  new Rotation3d(0, 0, 180)));
            put(27, new Pose3d(new Translation3d(4.2824, 3.4293, 1.1240),  new Rotation3d(0, 0, 270)));
            put(28, new Pose3d(new Translation3d(4.5883, 0.6444, 0.8890),  new Rotation3d(0, 0, 180)));
            put(29, new Pose3d(new Translation3d(0.0076, 0.6711, 0.5525),  new Rotation3d(0, 0, 0)));
            put(30, new Pose3d(new Translation3d(0.0076, 1.0978, 0.5525),  new Rotation3d(0, 0, 0)));
            put(31, new Pose3d(new Translation3d(0.0081, 3.7490, 0.5525),  new Rotation3d(0, 0, 0)));
            put(32, new Pose3d(new Translation3d(0.0081, 4.1835, 0.5525),  new Rotation3d(0, 0, 0)));
        }
    };

    public enum LimelightConstants {
        // 0.1812 x
        LIMELIGHT_LEFT("limelight-l", 0.1842, 0.172, 0.03, 26, 12.5, 82, 56.2),
        LIMELIGHT_RIGHT("limelight-r", 0.1842, 0.172, 0.0808, 26, -12.5, 82, 56.2),
        LIMELIGHT_OBJECT_DETECTION("limelight-o", 0.86, 0.245, 0.26, 40, 18, 62.5, 49);

        public final String NAME;

        // From floor
        public final double HEIGHT;
        public final double OFFSET_X;
        public final double OFFSET_Y;

        public final double PITCH;
        public final double YAW;

        public final double FOV_HORIZONTAL;
        public final double FOV_VERTICAL;


        LimelightConstants(String name, double height, double offsetX, double offsetY,
                           double pitch, double yaw, double fovHorizontal, double fovVertical) {
            NAME = name;
            HEIGHT = height;
            OFFSET_X = offsetX;
            OFFSET_Y = offsetY;
            PITCH = pitch;
            YAW = yaw;
            FOV_HORIZONTAL = fovHorizontal;
            FOV_VERTICAL = fovVertical;
        }
    }

    public enum CropStates {
        NONE,
        SIDE,
        BOTTOM
    }
}
