package frc.robot.subsystems.vision.visionLocalization;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionUtils;

import java.util.Optional;

import static edu.wpi.first.math.MathUtil.interpolate;
import static frc.robot.subsystems.localization.LocalizationConstants.*;
import static frc.robot.subsystems.vision.VisionConstants.APRIL_TAG_MAP;
import static frc.robot.subsystems.vision.VisionConstants.SMALLEST_INTEGER_BIGGER_THAN_ONE;
import static frc.robot.subsystems.vision.VisionUtils.isInField;

public class Limelight implements LocalizationService {
    private final VisionConstants.LimelightConstants limelightConstants;
    private final MjpegServer cameraServer;

    private Optional<VisionMeasurement> latestMeasurement;
    private Optional<Rotation2d> latestRotationEstimate;

    private LimelightHelpers.PoseEstimate latestPoseEstimate;

    private double latestDevs;

    public Limelight(VisionConstants.LimelightConstants limelightConstants) {
        this.limelightConstants = limelightConstants;
        this.cameraServer = CameraServer.startAutomaticCapture(new HttpCamera(
                limelightConstants.NAME,
                "http://" + limelightConstants.NAME + ".local:5800",
                HttpCamera.HttpCameraKind.kMJPGStreamer
        ));


        latestMeasurement = Optional.empty();
        latestDevs = Double.POSITIVE_INFINITY;

        LimelightHelpers.setLimelightNTDouble(limelightConstants.NAME, "throttle_set", 0);

    }

    @Override
    public double getYawFromTarget() {
        return LimelightHelpers.getTX(limelightConstants.NAME);
    }

    @Override
    public double getPitchFromTarget() {
        return LimelightHelpers.getTY(limelightConstants.NAME);
    }

    @Override
    public double getDistanceFromTarget() {
        LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightConstants.NAME);
        return estimate.avgTagDist;
    }

    @Override
    public boolean hasTarget() {
        return LimelightHelpers.getTV(limelightConstants.NAME);
    }

    @Override
    public String getName() {
        return limelightConstants.NAME;
    }

    @Override
    public MjpegServer getCameraStream() {
        return cameraServer;
    }

    private Optional<VisionMeasurement> calculateMeasurement(Pose2d currentBotPose) {
        latestPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightConstants.NAME);
        if (!shouldUpdate(latestPoseEstimate))
            return Optional.empty();

        latestDevs = calculateStDDevs(latestPoseEstimate, currentBotPose);

        return Optional.of(
                new VisionMeasurement(
                        latestPoseEstimate.pose,
                        latestPoseEstimate.timestampSeconds,
                        latestDevs
                )
        );
    }

    private Optional<Rotation2d> calculateRotationMeasurement() {
        var rotationalMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightConstants.NAME);
        if (rotationalMeasurement == null || !shouldUpdateRotation(rotationalMeasurement))
            return Optional.empty();
        else
            return Optional.of(rotationalMeasurement.pose.getRotation());
    }

    private boolean shouldUpdateRotation(LimelightHelpers.PoseEstimate rotationalMeasurement) {
        if (!RobotState.isDisabled())
            return false;
        return rotationalMeasurement.tagCount >= 2;
    }

    @Override
    public void update(Pose2d currentBotPose) {
        setRobotOrientation(currentBotPose.getRotation().getDegrees());
        latestMeasurement = calculateMeasurement(currentBotPose);
        latestRotationEstimate = calculateRotationMeasurement();

//        if (RobotState.isDisabled()) {
//            LimelightHelpers.setLimelightNTDouble(limelightConstants.NAME, "throttle_set", 100);
//        } else {
//            LimelightHelpers.setLimelightNTDouble(limelightConstants.NAME, "throttle_set", 0);
//        }

    }

    @Override
    public Optional<VisionMeasurement> getLatestMeasurement() {
        return latestMeasurement;
    }

    @Override
    public Optional<Rotation2d> getLatestRotationMeasurement() {
        return latestRotationEstimate;
    }

    @Override
    public void setPipeline(int pipelineID) {
        LimelightHelpers.setPipelineIndex(limelightConstants.NAME, pipelineID);
    }

    @Override
    public void setRobotOrientation(double angle) {
        LimelightHelpers.SetRobotOrientation(limelightConstants.NAME, angle, 0, 0, 0, 0, 0);
    }

    @Override
    public int getPipeline() {
        return (int) LimelightHelpers.getCurrentPipelineIndex(limelightConstants.NAME);
    }

    @Override
    public int getTargetID() {
        return (int) LimelightHelpers.getFiducialID(limelightConstants.NAME);
    }

    private double calculateStDDevs(LimelightHelpers.PoseEstimate estimate, Pose2d botPose) {
        double devs;
        if (estimate.avgTagDist > MAXIMUM_INTERPOLATION_DISTANCE) {
            devs = WORST_CASE_DEVS;
        } else if (estimate.tagCount >= SMALLEST_INTEGER_BIGGER_THAN_ONE) {
            devs = VisionUtils.interpolate(estimate.avgTagDist, MULTI_TAG_MIN_DEVS, MULTI_TAG_MAX_DEVS, MINIMUM_INTERPOLATION_DISTANCE, MAXIMUM_INTERPOLATION_DISTANCE);
        } else {
            devs = VisionUtils.interpolate(estimate.avgTagDist, SINGLE_TAG_MIN_DEVS, SINGLE_TAG_MAX_DEVS, MINIMUM_INTERPOLATION_DISTANCE, MAXIMUM_INTERPOLATION_DISTANCE);
        }
        double estimateToRobotDistance = Math.abs(botPose.minus(estimate.pose).getTranslation().getNorm());

        estimateToRobotDistance = MathUtil.clamp(estimateToRobotDistance, CLOSEST_INTERPOLATED_DISTANCE, FURTHEST_INTERPOLATED_DISTANCE);
        double val = Math.abs(1 - MathUtil.inverseInterpolate(CLOSEST_INTERPOLATED_DISTANCE, FURTHEST_INTERPOLATED_DISTANCE, estimateToRobotDistance));
        devs *= interpolate(FURTHEST_MULTIPLIER, CLOSEST_MULTIPLIER, val);

        devs = MathUtil.clamp(devs, BEST_CASE_DEVS, WORST_CASE_DEVS);

        return devs;
    }

    @Override
    public void log() {
        if (latestPoseEstimate != null) {

        }
    }

    private Pose3d[] getVisibleTags() {
        LimelightHelpers.RawFiducial[] fiducial = LimelightHelpers.getRawFiducials(limelightConstants.NAME);
        Pose3d[] positions = new Pose3d[fiducial.length];
        for (int i = 0; i < fiducial.length; i++) {
            int id = fiducial[i].id;

            Pose3d pose = APRIL_TAG_MAP.get(id);
            if (pose != null) {
                positions[i] = pose;
            } else {
                positions[i] = new Pose3d(Double.NaN, Double.NaN, Double.NaN, new Rotation3d());
            }
        }
        return positions;
    }

    private boolean shouldUpdate(LimelightHelpers.PoseEstimate estimate) {
        if (estimate == null) {
            return false;
        }

        return isInField(estimate.pose, 0);
    }
}
