package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.lib.PID.PIDEntries;
import frc.robot.lib.PhysicalTelemetryEntries;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.hood.HoodConstants.*;

public class HoodShuffleboard {
    public HoodShuffleboard() {
        String name = HOOD_SUBSYSTEM_NAME;
        ShuffleboardTab tab = Shuffleboard.getTab(name);

        PhysicalTelemetryEntries hood = new PhysicalTelemetryEntries(name, HOOD_SUBSYSTEM_NAME,
            () -> Hood.getInstance().getAngle(),
            () -> Hood.getInstance().getVelocity(),
            () -> Hood.getInstance().getAcceleration()
        );

        PIDEntries hoodEntries = new PIDEntries(name, HOOD_SUBSYSTEM_NAME, SIMULATION_HOOD_PID_VALUES);
        tab.add("update hood pid", new InstantCommand(() -> Hood.getInstance()
            .updatePID(hoodEntries.getPIDValues())));

        tab.addString("Wanted state", () -> Hood.getInstance().getWantedState().toString());
        tab.addString("System state", () -> Hood.getInstance().getSystemState().toString());

        DoubleSupplier targetAngle = () -> hood.getTargetPosition();

        tab.add("idle", new InstantCommand(() -> Hood.getInstance().setWantedState(Hood.WantedState.IDLE)));
        tab.add("move to angle", new InstantCommand(() -> Hood.getInstance().setWantedState(Hood.WantedState.MOVE_TO_ANGLE, targetAngle.getAsDouble())));
        tab.add("move forwards", new InstantCommand(() -> Hood.getInstance().setWantedState(Hood.WantedState.MOVE_FORWARDS)));
    }
}
