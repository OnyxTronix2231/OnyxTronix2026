package frc.robot.objectDetection;

public class ObjectDetectionConstants {
    public static final double BALL_RADIUS = 0.150114;

    public static final double PIXEL_AMOUNT_LENGTH = 640;
    public static final double PIXEL_AMOUNT_WIDTH = 480;

    public static final double WIDTH_FOV = 62.5;
    public static final double HEIGHT_FOV = 49;

    public static final double CAMERA_HEIGHT = 0.86742852146;

    public static final double CAMERA_PITCH = 37;

    public static double PIXELS_TO_DEGREES(double pixelAmount, double pixelRange, double fov) {
        double percentage = (pixelAmount / pixelRange);

        return percentage * fov;
    }

    public static double DEGREES_TO_PIXELS(double degreeAmount, double degreeRange, double pixelRange) {
        double percentage = (degreeAmount / degreeRange);

        return Math.round(percentage * pixelRange);
    }
}
