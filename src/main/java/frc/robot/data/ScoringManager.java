package frc.robot.data;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.localization.Localization;

import static frc.robot.data.FieldConstants.*;
import static frc.robot.data.FieldUtils.*;


public class ScoringManager {

    private ScoringData.BallShootingType ballShootingType;
    private ScoringData.ClimbingSide climbingSide;

    public ScoringManager() {
        ballShootingType = ScoringData.BallShootingType.SCORE;
        climbingSide = ScoringData.ClimbingSide.RIGHT;
    }

    public ScoringData.BallShootingType getBallShootingType() {
        return ballShootingType;
    }

    public void setBallShootingType(ScoringData.BallShootingType ballShootingType) {
        this.ballShootingType = ballShootingType;
    }

    public ScoringData.ClimbingSide getClimbingSide() {
        return climbingSide;
    }

    public void setClimbingSide(ScoringData.ClimbingSide climbingSide) {
        this.climbingSide = climbingSide;
    }

    public Pose2d getNearestPos(Pose2d pos1, Pose2d pos2) {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        Pose2d botPose = Localization.getInstance().getBotPose();
        if (alliance == DriverStation.Alliance.Blue) {
            return nearestPos(botPose, pos1, pos2);
        } else {
            return nearestPos(botPose,flipPose(pos1), flipPose(pos2));
        }
    }

    public boolean isRobotInRect(double x, double y, Pose2d centerOfRect){
        Pose2d botPose = Localization.getInstance().getBotPose();
        return isInRect(botPose,x,y,centerOfRect);
    }

    public Pose2d getNearestBump() {
        return getNearestPos(BLUE_RIGHT_BUMP,BLUE_LEFT_BUMP);
    }

    public boolean isOnNearestBump() {
        return isRobotInRect(BUMP_LENGTH_X,BUMP_LENGTH_Y,getNearestBump());
    }

    public Pose2d getNearestTrench() {
        return getNearestPos(BLUE_RIGHT_TRENCH,BLUE_LEFT_TRENCH);
    }

    public boolean isUnderNearestTrench() {
        return isRobotInRect(TRENCH_LENGTH_X,TRENCH_LENGTH_Y,getNearestTrench());
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

    public Pose2d getClimbingTarget(){
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        if (alliance == DriverStation.Alliance.Blue) {
            if (climbingSide == ScoringData.ClimbingSide.RIGHT)
                return BLUE_RIGHT_TOWER;
            return BLUE_LEFT_TOWER;
        }
        else{
            if (climbingSide == ScoringData.ClimbingSide.RIGHT)
                return flipPose(BLUE_RIGHT_TOWER);
            return flipPose(BLUE_LEFT_TOWER);
        }
    }

    private Pose2d getPoseBasedOnCondition(boolean condition, Pose2d pose, double distanceX, double distanceY) {
        if (condition)
            return pose;
        else
            return pose.transformBy(new Transform2d(-distanceX, distanceY, new Rotation2d(0)));
    }

    public static boolean climbingCondition = false;

    public Pose2d getClimbingTargetBasedOnCondition() {
        double distanceY = climbingSide == ScoringData.ClimbingSide.RIGHT ? -CLIMBING_DISTANCE : CLIMBING_DISTANCE;
        return getPoseBasedOnCondition(climbingCondition,
                getClimbingTarget(), 0, distanceY);
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
