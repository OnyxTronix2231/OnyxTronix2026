package frc.robot.data;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.lib.Point;
import frc.robot.lib.VectorMath.VectorMathUtils;
import frc.robot.subsystems.localization.Localization;
import org.littletonrobotics.junction.Logger;

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

    public Pose2d getNearestBump() {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        Pose2d botPose = Localization.getInstance().getBotPose();
        if (alliance == DriverStation.Alliance.Blue) {
            return nearestPos(botPose, BLUE_RIGHT_BUMP, BLUE_LEFT_BUMP);
        } else {
            return nearestPos(botPose, flipPose(BLUE_RIGHT_BUMP), flipPose(BLUE_LEFT_BUMP));
        }
    }

    public boolean isOnNearestBump() {
        Pose2d botPose = Localization.getInstance().getBotPose();
        Pose2d bumpCenter = getNearestBump();
        return isInRect(botPose, BUMP_LENGTH_X, BUMP_LENGTH_Y, bumpCenter);
    }

    public Pose2d getNearestTrench() {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        Pose2d botPose = Localization.getInstance().getBotPose();
        if (alliance == DriverStation.Alliance.Blue) {
            return nearestPos(botPose, BLUE_RIGHT_TRENCH, BLUE_LEFT_TRENCH);
        } else {
            return nearestPos(botPose, flipPose(BLUE_RIGHT_TRENCH), flipPose(BLUE_LEFT_TRENCH));
        }
    }

    public boolean isUnderNearestTrench() {
        Pose2d botPose = Localization.getInstance().getBotPose();
        Pose2d trenchCenter = getNearestTrench();
        return isInRect(botPose, TRENCH_LENGTH_X, TRENCH_LENGTH_Y, trenchCenter);
    }

    public boolean isInHub(Translation2d pos) {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        Pose2d center = alliance == DriverStation.Alliance.Blue ? BLUE_HUB.toPose2d() : flipPose(BLUE_HUB.toPose2d());
        return isInRect(new Pose2d(pos, Rotation2d.fromDegrees(0)), HUB_LENGTH, HUB_LENGTH, center);
    }

    public boolean isInHubs(Translation2d pos) {
        return isInRect(new Pose2d(pos, Rotation2d.fromDegrees(0)), HUB_LENGTH, HUB_LENGTH, BLUE_HUB.toPose2d())
                || isInRect(new Pose2d(pos, Rotation2d.fromDegrees(0)), HUB_LENGTH, HUB_LENGTH, flipPose(BLUE_HUB.toPose2d()));
    }

    public Pose2d getDeliveryTarget() {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        Pose2d botPose = Localization.getInstance().getBotPose();
        if (alliance == DriverStation.Alliance.Blue) {
            if (botPose.getTranslation().getY() <= FIELD_MAX_Y / 2) {
                return BLUE_RIGHT_DELIVERY;
            }
            return BLUE_LEFT_DELIVERY;
        } else {
            if (botPose.getTranslation().getY() <= FIELD_MAX_Y / 2) {
                return flipPose(BLUE_RIGHT_DELIVERY);
            }
            return flipPose(BLUE_LEFT_DELIVERY);
        }
    }

    public Pose2d getClimbingTarget() {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        if (alliance == DriverStation.Alliance.Blue) {
            if (climbingSide == ScoringData.ClimbingSide.RIGHT)
                return BLUE_RIGHT_TOWER;
            return BLUE_LEFT_TOWER;
        } else {
            if (climbingSide == ScoringData.ClimbingSide.RIGHT)
                return flipPose(BLUE_RIGHT_TOWER);
            return flipPose(BLUE_LEFT_TOWER);
        }
    }

    public boolean rightCanDelivery() {
        Pose2d botPose = Localization.getInstance().getBotPose();
        Pose2d deliveryPos;

        Translation2d blueBottomRightCorner;
        Translation2d blueBottomLeftCorner;
        Translation2d blueTopRightCorner;
        Translation2d blueTopLeftCorner;
        Translation2d redBottomRightCorner;
        Translation2d redBottomLeftCorner;
        Translation2d redTopRightCorner;
        Translation2d redTopLeftCorner;

        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Blue)) {
            deliveryPos = BLUE_RIGHT_DELIVERY;
        } else {
            deliveryPos = flipPose((BLUE_RIGHT_DELIVERY));
        }
        blueBottomRightCorner = BLUE_HUB_BOTTOM_RIGHT;
        blueBottomLeftCorner = BLUE_HUB_BOTTOM_LEFT;
        blueTopRightCorner = BLUE_HUB_TOP_RIGHT;
        blueTopLeftCorner = BLUE_HUB_TOP_LEFT;

        redBottomRightCorner = flipPose(new Pose2d(BLUE_HUB_BOTTOM_RIGHT, Rotation2d.fromDegrees(0))).getTranslation();
        redBottomLeftCorner = flipPose(new Pose2d(BLUE_HUB_BOTTOM_LEFT, Rotation2d.fromDegrees(0))).getTranslation();
        redTopRightCorner = flipPose(new Pose2d(BLUE_HUB_TOP_RIGHT, Rotation2d.fromDegrees(0))).getTranslation();
        redTopLeftCorner = flipPose(new Pose2d(BLUE_HUB_TOP_LEFT, Rotation2d.fromDegrees(0))).getTranslation();


        Point pointA = VectorMathUtils.findIntersectionPoint(
                VectorMathUtils.getFunction(
                        new Point(botPose.getX(), botPose.getY()), new Point(deliveryPos.getX(), deliveryPos.getY())),
                VectorMathUtils.getFunction(
                        new Point(blueBottomLeftCorner.getX(), blueBottomLeftCorner.getY()), new Point(blueTopRightCorner.getX(), blueTopRightCorner.getY())
                )
        );
        Point pointB = VectorMathUtils.findIntersectionPoint(
                VectorMathUtils.getFunction(
                        new Point(botPose.getX(), botPose.getY()), new Point(deliveryPos.getX(), deliveryPos.getY())),
                VectorMathUtils.getFunction(
                        new Point(blueBottomRightCorner.getX(), blueBottomRightCorner.getY()), new Point(blueTopLeftCorner.getX(), blueTopLeftCorner.getY())
                )
        );

        Point pointC = VectorMathUtils.findIntersectionPoint(
                VectorMathUtils.getFunction(
                        new Point(botPose.getX(), botPose.getY()), new Point(deliveryPos.getX(), deliveryPos.getY())),
                VectorMathUtils.getFunction(
                        new Point(redBottomLeftCorner.getX(), redBottomLeftCorner.getY()), new Point(redTopRightCorner.getX(), redTopRightCorner.getY())
                )
        );
        Point pointD = VectorMathUtils.findIntersectionPoint(
                VectorMathUtils.getFunction(
                        new Point(botPose.getX(), botPose.getY()), new Point(deliveryPos.getX(), deliveryPos.getY())),
                VectorMathUtils.getFunction(
                        new Point(redBottomRightCorner.getX(), redBottomRightCorner.getY()), new Point(redTopLeftCorner.getX(), redTopLeftCorner.getY())
                )
        );


        Logger.recordOutput("RightDeliveryPoint", deliveryPos);

        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Blue))
            return !(isInHubs(new Translation2d(pointA.x(), pointA.y())) || isInHubs(new Translation2d(pointB.x(), pointB.y())) || isInHubs(new Translation2d(pointC.x(), pointC.y())) || isInHubs(new Translation2d(pointD.x(), pointD.y()))) || botPose.getX() < blueBottomRightCorner.getX();
        return !(isInHubs(new Translation2d(pointA.x(), pointA.y())) || isInHubs(new Translation2d(pointB.x(), pointB.y())) || isInHubs(new Translation2d(pointC.x(), pointC.y())) || isInHubs(new Translation2d(pointD.x(), pointD.y()))) || botPose.getX() > redBottomRightCorner.getX();
    }

    public boolean leftCanDelivery() {
        Pose2d botPose = Localization.getInstance().getBotPose();
        Pose2d deliveryPos;

        Translation2d blueBottomRightCorner;
        Translation2d blueBottomLeftCorner;
        Translation2d blueTopRightCorner;
        Translation2d blueTopLeftCorner;
        Translation2d redBottomRightCorner;
        Translation2d redBottomLeftCorner;
        Translation2d redTopRightCorner;
        Translation2d redTopLeftCorner;

        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Blue)) {
            deliveryPos = BLUE_LEFT_DELIVERY;
        } else {
            deliveryPos = flipPose((BLUE_LEFT_DELIVERY));
        }
        blueBottomRightCorner = BLUE_HUB_BOTTOM_RIGHT;
        blueBottomLeftCorner = BLUE_HUB_BOTTOM_LEFT;
        blueTopRightCorner = BLUE_HUB_TOP_RIGHT;
        blueTopLeftCorner = BLUE_HUB_TOP_LEFT;

        redBottomRightCorner = flipPose(new Pose2d(BLUE_HUB_BOTTOM_RIGHT, Rotation2d.fromDegrees(0))).getTranslation();
        redBottomLeftCorner = flipPose(new Pose2d(BLUE_HUB_BOTTOM_LEFT, Rotation2d.fromDegrees(0))).getTranslation();
        redTopRightCorner = flipPose(new Pose2d(BLUE_HUB_TOP_RIGHT, Rotation2d.fromDegrees(0))).getTranslation();
        redTopLeftCorner = flipPose(new Pose2d(BLUE_HUB_TOP_LEFT, Rotation2d.fromDegrees(0))).getTranslation();


        Point pointA = VectorMathUtils.findIntersectionPoint(
                VectorMathUtils.getFunction(
                        new Point(botPose.getX(), botPose.getY()), new Point(deliveryPos.getX(), deliveryPos.getY())),
                VectorMathUtils.getFunction(
                        new Point(blueBottomLeftCorner.getX(), blueBottomLeftCorner.getY()), new Point(blueTopRightCorner.getX(), blueTopRightCorner.getY())
                )
        );
        Point pointB = VectorMathUtils.findIntersectionPoint(
                VectorMathUtils.getFunction(
                        new Point(botPose.getX(), botPose.getY()), new Point(deliveryPos.getX(), deliveryPos.getY())),
                VectorMathUtils.getFunction(
                        new Point(blueBottomRightCorner.getX(), blueBottomRightCorner.getY()), new Point(blueTopLeftCorner.getX(), blueTopLeftCorner.getY())
                )
        );

        Point pointC = VectorMathUtils.findIntersectionPoint(
                VectorMathUtils.getFunction(
                        new Point(botPose.getX(), botPose.getY()), new Point(deliveryPos.getX(), deliveryPos.getY())),
                VectorMathUtils.getFunction(
                        new Point(redBottomLeftCorner.getX(), redBottomLeftCorner.getY()), new Point(redTopRightCorner.getX(), redTopRightCorner.getY())
                )
        );
        Point pointD = VectorMathUtils.findIntersectionPoint(
                VectorMathUtils.getFunction(
                        new Point(botPose.getX(), botPose.getY()), new Point(deliveryPos.getX(), deliveryPos.getY())),
                VectorMathUtils.getFunction(
                        new Point(redBottomRightCorner.getX(), redBottomRightCorner.getY()), new Point(redTopLeftCorner.getX(), redTopLeftCorner.getY())
                )
        );


        Logger.recordOutput("leftDeliveryPoint", deliveryPos);

        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Blue))
            return !(isInHubs(new Translation2d(pointA.x(), pointA.y())) || isInHubs(new Translation2d(pointB.x(), pointB.y())) || isInHubs(new Translation2d(pointC.x(), pointC.y())) || isInHubs(new Translation2d(pointD.x(), pointD.y()))) || botPose.getX() < blueBottomRightCorner.getX();
        return !(isInHubs(new Translation2d(pointA.x(), pointA.y())) || isInHubs(new Translation2d(pointB.x(), pointB.y())) || isInHubs(new Translation2d(pointC.x(), pointC.y())) || isInHubs(new Translation2d(pointD.x(), pointD.y()))) || botPose.getX() > redBottomRightCorner.getX();
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
