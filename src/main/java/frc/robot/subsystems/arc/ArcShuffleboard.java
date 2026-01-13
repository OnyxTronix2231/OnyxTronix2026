package frc.robot.subsystems.arc;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.lib.PID.PIDEntries;
import frc.robot.lib.PhysicalTelemetryEntries;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.arc.ArcConstants.SIMULATION_ARC_PID_VALUES;

public class ArcShuffleboard {
    public ArcShuffleboard() {
        String name = "Arc";
        ShuffleboardTab tab = Shuffleboard.getTab(name);

        PhysicalTelemetryEntries arc = new PhysicalTelemetryEntries(name, "Arc",
            () -> Arc.getInstance().getAngle(),
            () -> Arc.getInstance().getVelocity(),
            () -> Arc.getInstance().getAcceleration()
        );

        PIDEntries arcEntries = new PIDEntries(name, "Arc", SIMULATION_ARC_PID_VALUES);
        tab.add("update elevator pid", new InstantCommand(() -> Arc.getInstance()
            .updatePID(arcEntries.getPIDValues())));

        tab.addString("Wanted state", () -> Arc.getInstance().getWantedState().toString());
        tab.addString("System state", () -> Arc.getInstance().getSystemState().toString());

        DoubleSupplier targetAngle = () -> arc.getTargetPosition();

        tab.add("idle", new InstantCommand(() -> Arc.getInstance().setWantedState(Arc.WantedState.IDLE)));
        tab.add("move to angle", new InstantCommand(() -> Arc.getInstance().setWantedState(Arc.WantedState.MOVE_TO_ANGLE, targetAngle.getAsDouble())));
    }
}
