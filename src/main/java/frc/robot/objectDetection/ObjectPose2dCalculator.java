package frc.robot.objectDetection;

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



}
