package frc.robot.subsystems.vision.visionLocalization;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;

import static frc.robot.subsystems.localization.LocalizationConstants.VISION_WEIGHT_YAW;

public class VisionMeasurement {

    public final Pose2d pose;
    public final double timestamp;

    public final Vector<N3> stdDevs;

    public VisionMeasurement(Pose2d pose, double timestamp, double xyDeviation) {
        this.pose = pose;
        this.timestamp = timestamp;
        this.stdDevs = VecBuilder.fill(xyDeviation, xyDeviation, VISION_WEIGHT_YAW);
    }
}
