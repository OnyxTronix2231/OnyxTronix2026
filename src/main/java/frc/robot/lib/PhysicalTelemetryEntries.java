package frc.robot.lib;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.util.function.DoubleSupplier;

public class PhysicalTelemetryEntries {

    private GenericEntry targetPosition;

    public PhysicalTelemetryEntries(String tabName, String prefix, DoubleSupplier positionSupplier, DoubleSupplier velocitySupplier, DoubleSupplier accelerationSupplier) {
        ShuffleboardTab tab = Shuffleboard.getTab(tabName);

        tab.addDouble(prefix + " position", positionSupplier);
        tab.addDouble(prefix + " velocity", velocitySupplier);
        tab.addDouble(prefix + " acceleration", accelerationSupplier);

        this.targetPosition = tab.add(prefix + " target position", 0).getEntry();
        tab.addDouble(prefix + " position error", () -> positionSupplier.getAsDouble() - targetPosition.getDouble(0));
    }

    public double getTargetPosition() {
        return targetPosition.getDouble(0);
    }
}
