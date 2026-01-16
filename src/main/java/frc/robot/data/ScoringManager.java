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

    public boolean isOnNearestBump() {
        Pose2d botPose = Localization.getInstance().getBotPose();
        Pose2d bumpCenter = getNearestBump();
        return isInRect(botPose,BUMP_LENGTH_X,BUMP_LENGTH_Y,bumpCenter);
    }

    public Pose2d getNearestTrench() {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        Pose2d botPose = Localization.getInstance().getBotPose();
        if (alliance == DriverStation.Alliance.Blue) {
            return nearestPos(botPose, BLUE_RIGHT_TRENCH, BLUE_LEFT_TRENCH);
        } else {
            return nearestPos(botPose,flipPose(BLUE_RIGHT_TRENCH), flipPose(BLUE_LEFT_TRENCH));
        }
    }

    public boolean isUnderNearestTrench() {
        Pose2d botPose = Localization.getInstance().getBotPose();
        Pose2d trenchCenter = getNearestTrench();
        return isInRect(botPose,TRENCH_LENGTH_X,TRENCH_LENGTH_Y,trenchCenter);
    }

    public boolean isInHub(Translation2d pos) {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        Pose2d center = alliance == DriverStation.Alliance.Blue ? BLUE_HUB.toPose2d() : flipPose(BLUE_HUB.toPose2d());
        return isInRect(new Pose2d(pos, Rotation2d.fromDegrees(0)), HUB_LENGTH, HUB_LENGTH, center);
    }

    public Pose2d getDeliveryTarget(){
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        Pose2d botPose = Localization.getInstance().getBotPose();
        if (alliance == DriverStation.Alliance.Blue) {
            if (botPose.getTranslation().getY() <= FIELD_MAX_Y/2) {
                return BLUE_RIGHT_DELIVERY;
            }
            return BLUE_LEFT_DELIVERY;
        }
        else{
            if (botPose.getTranslation().getY() <= FIELD_MAX_Y/2) {
                return flipPose(BLUE_RIGHT_DELIVERY);
            }
            return flipPose(BLUE_LEFT_DELIVERY);
        }
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
