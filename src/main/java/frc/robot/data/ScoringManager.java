package frc.robot.data;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.localization.Localization;

import java.util.List;

import static frc.robot.data.FieldConstants.*;


public class ScoringManager {

    public enum BallShootingType{
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
            return botPose.nearest(List.of(BLUE_RIGHT_BUMP, BLUE_LEFT_BUMP));
        } else {
            return botPose.nearest(List.of(flipPose(BLUE_RIGHT_BUMP), flipPose(BLUE_LEFT_BUMP)));
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
