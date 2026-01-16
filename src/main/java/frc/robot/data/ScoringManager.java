package frc.robot.data;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.localization.Localization;

import static frc.robot.data.FieldConstants.*;
import static frc.robot.data.FieldUtils.*;


public class ScoringManager {

    public enum BallShootingType {
        SCORE,
        DELIVERY
    }

    private BallShootingType ballShootingType;

    public ScoringManager() {
        ballShootingType = BallShootingType.SCORE;
    }

    public BallShootingType getBallShootingType() {
        return ballShootingType;
    }

    public void setBallShootingType(BallShootingType ballShootingType) {
        this.ballShootingType = ballShootingType;
    }

    public Pose2d getNearestBump() {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        Pose2d botPose = Localization.getInstance().getBotPose();
        if (alliance == DriverStation.Alliance.Blue) {
            return nearestPos(botPose, BLUE_RIGHT_BUMP, BLUE_LEFT_BUMP);
        } else {
            return nearestPos(botPose,flipPose(BLUE_RIGHT_BUMP), flipPose(BLUE_LEFT_BUMP));
        }
    }

    public boolean isInHub(Translation2d pos) {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        Pose2d center = alliance == DriverStation.Alliance.Blue ? BLUE_HUB.toPose2d() : flipPose(BLUE_HUB.toPose2d());
        return isInRect(new Pose2d(pos, Rotation2d.fromDegrees(0)), HUB_LENGTH, HUB_LENGTH, center);
    }

    public boolean canDeliver(Pose2d deliveryPose) {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        Pose2d botPose = Localization.getInstance().getBotPose();
        Pose2d hubPose = BLUE_HUB.toPose2d();
        if (alliance == DriverStation.Alliance.Red) {
            hubPose = flipPose(BLUE_HUB.toPose2d());
        }

        Translation2d vector = new Translation2d(deliveryPose.getX() - botPose.getX(), deliveryPose.getY() - botPose.getY());
        Translation2d hub_vector = new Translation2d(hubPose.getX() - botPose.getX(), hubPose.getY() - botPose.getY());

        double length = hub_vector.getNorm();
        vector = new Translation2d(vector.getX() * (length / vector.getNorm()), vector.getY() * (length / vector.getNorm()));
        return !isInHub(vector.minus(botPose.getTranslation()).times(-1));

    }

    private static ScoringManager instance;

    public static void init() {
        if (instance == null)
            instance = new ScoringManager();
    }

    public static ScoringManager getInstance() {
        return instance;
    }
}
