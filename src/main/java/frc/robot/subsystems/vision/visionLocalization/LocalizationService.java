package frc.robot.subsystems.vision.visionLocalization;

import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Optional;

public interface LocalizationService {

    int getPipeline();

    int getTargetID();

    double getYawFromTarget();

    double getPitchFromTarget();

    double getDistanceFromTarget();

    boolean hasTarget();

    String getName();

    MjpegServer getCameraStream();

    void update(Pose2d currentBotPose);

    Optional<VisionMeasurement> getLatestMeasurement();

    Optional<Rotation2d> getLatestRotationMeasurement();

    void setPipeline(int pipelineID);

    void setRobotOrientation(double angle);

    void log();
}
