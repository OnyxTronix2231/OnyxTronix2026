package frc.robot.visualization;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

import java.util.ArrayList;
import java.util.List;

public abstract class VisualizedSubsystem {

    public static final LoggedMechanism2d ROBOT_MECHANISM = new LoggedMechanism2d(6.2, 4);

    private static final List<VisualizedSubsystem> subsystems = new ArrayList<>();

    public VisualizedSubsystem() {
        subsystems.add(this);
    }

    public static void updateVisualizations() {
        for (VisualizedSubsystem subsystem : subsystems) {
            subsystem.updateVisualization();
        }
        Logger.recordOutput("RobotMechanism", ROBOT_MECHANISM);
    }

    abstract void updateVisualization();
}
