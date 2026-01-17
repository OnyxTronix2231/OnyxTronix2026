package frc.robot.objectDetection;

import static frc.robot.objectDetection.ObjectDetectionConstants.BALL_RADIUS;
import static frc.robot.objectDetection.ObjectDetectionConstants.CAMERA_HEIGHT;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class ObjectPose2dCalculator {

    String limelightName;

    public ObjectPose2dCalculator(String limelightName) {
        this.limelightName = limelightName;
    }

    public double getTX() {
        return NetworkTableInstance.getDefault().getTable(limelightName).getEntry("llpython").getDoubleArray(new double[4])[0];
    }

    public double getTY() {
        return NetworkTableInstance.getDefault().getTable(limelightName).getEntry("llpython").getDoubleArray(new double[4])[1];
    }

    public double getThor() {
        return NetworkTableInstance.getDefault().getTable(limelightName).getEntry("llpython").getDoubleArray(new double[4])[14];
    }

    public double calculateDistance(){
        return (BALL_RADIUS/2)/Math.tan(getThor()/2);
    }

    public Pose3d calculateBallPos() {
        double centerLineLength = calculateDistance()*Math.cos(getTX());
        double floorLineLength = Math.sqrt(centerLineLength*centerLineLength-CAMERA_HEIGHT*CAMERA_HEIGHT);
        double floorOffsetLength = calculateDistance()*Math.sin(getTX());
        double floorTX = Math.atan2(floorOffsetLength, floorLineLength);
        double objX = floorLineLength*Math.cos(floorTX);
        double objY = floorLineLength*Math.sin(floorTX);
        return new Pose3d(new Translation3d(objX,objY,BALL_RADIUS/2), Rotation3d.kZero);
    }

}
