package frc.robot.subsystems.localization;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import data.OnyxGenericValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.visionLocalization.Limelight;
import frc.robot.subsystems.vision.visionLocalization.LocalizationService;
import frc.robot.subsystems.vision.visionLocalization.VisionMeasurement;

import static frc.robot.subsystems.localization.LocalizationConstants.ROBOT_YAW_THRESHOLD;

public class Localization extends SubsystemBase {
    private final CommandSwerveDrivetrain swerveDrivetrain;
    private final SwerveDrivetrain.SwerveDriveState swerveState;
    private final Pigeon2 pigeon;

    private final LocalizationService[] localizationServices;
    OnyxGenericValue angularVal = new OnyxGenericValue("max angular value", "vision");

    private boolean isVisionUpdateOdometry;

    private Localization() {
        swerveDrivetrain = CommandSwerveDrivetrain.getInstance();
        swerveState = swerveDrivetrain.getState();

        pigeon = swerveDrivetrain.getPigeon2();

        isVisionUpdateOdometry = true;


        localizationServices = new LocalizationService[]{
                new Limelight(VisionConstants.LimelightConstants.LIMELIGHT_LEFT),
                new Limelight(VisionConstants.LimelightConstants.LIMELIGHT_RIGHT),
        };
    }

    @Override
    public void periodic() {
        for (LocalizationService service : localizationServices) {
            service.update(swerveState.Pose);

            if (service.hasTarget()) {
                addVisionMeasurement(service);
            }
            addRotationalMeasurement(service);
        }

        log();
    }

    private void log() {
        for (LocalizationService service : localizationServices) {
            service.log();
        }
    }

    public boolean hasTag() {
        for (LocalizationService localizationService : localizationServices) {
            if (localizationService.hasTarget()) {
                return true;
            }
        }
        return false;
    }

    private void addVisionMeasurement(LocalizationService service) {
        if (!isVisionUpdateOdometry) {
            return;
        }

        if (Math.abs(swerveState.Speeds.omegaRadiansPerSecond) > 2 * Math.PI)
            return;

        var optionalEstimate = service.getLatestMeasurement();
        if (optionalEstimate.isEmpty()) {
            return;
        }

        VisionMeasurement estimate = optionalEstimate.get();
        swerveDrivetrain.addVisionMeasurement(estimate.pose, Utils.fpgaToCurrentTime(estimate.timestamp), estimate.stdDevs);
    }

    private void addRotationalMeasurement(LocalizationService service) {

        var optionalEstimate = service.getLatestRotationMeasurement();
        if (optionalEstimate.isEmpty()) {
            return;
        }

        Rotation2d estimate = optionalEstimate.get();
        var currentBotPose = getBotPose();

        swerveDrivetrain.resetPose(
                new Pose2d(currentBotPose.getX(), currentBotPose.getY(), estimate)
        );
    }

    public void shouldUpdateOdometryByVision(boolean a) {
        isVisionUpdateOdometry = a;
    }

    public boolean isRobotYawOnTarget(double target) {
        return Math.abs(Localization.getInstance().getBotPose()
                .getRotation().minus(Rotation2d.fromDegrees(target)).getDegrees()) < ROBOT_YAW_THRESHOLD;
    }


    public Pose2d getBotPose() {
        return swerveState.Pose;
    }

    public LocalizationService[] getLocalizationServices() {
        return localizationServices;
    }

    public double getBotPitch() {
        return pigeon.getPitch().getValueAsDouble();
    }

    public double getBotRoll() {
        return pigeon.getRoll().getValueAsDouble();
    }

    private static Localization instance;

    public static Localization getInstance() {
        return instance;
    }

    public static void init() {
        if (instance == null) {
            instance = new Localization();
        }
    }
}
