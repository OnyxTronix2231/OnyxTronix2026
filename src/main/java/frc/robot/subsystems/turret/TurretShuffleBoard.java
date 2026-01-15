package frc.robot.subsystems.turret;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.lib.PID.PIDEntries;
import frc.robot.lib.PhysicalTelemetryEntries;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.turret.TurretConstants.*;

public class TurretShuffleBoard {
    public TurretShuffleBoard() {
        String subsystemName = TURRET_SUBSYSTEM_NAME;
        String motorName = TURRET_MOTOR_NAME;

        ShuffleboardTab tab = Shuffleboard.getTab(subsystemName);

        //PHYSICAL VALUES
        PhysicalTelemetryEntries turret = new PhysicalTelemetryEntries(subsystemName, motorName,
                () -> Turret.getInstance().getTurretAngle(),
                () -> Turret.getInstance().getTurretVelocity(),
                () -> Turret.getInstance().getTurretAcceleration()
        );
        DoubleSupplier targetAngle = () -> turret.getTargetPosition();
        
        //PID
        PIDEntries turretEntries = new PIDEntries(subsystemName, motorName, TURRET_PID_VALUES);
        tab.add("update turret pid", new InstantCommand(() -> Turret.getInstance().updateTurretPID(turretEntries.getPIDValues())));

        //STATES
        tab.addString("wanted state", () -> Turret.getInstance().getWantedState().toString());
        tab.addString("system state", () -> Turret.getInstance().getSystemState().toString());

        tab.add("idle", new InstantCommand(() -> Turret.getInstance().setWantedState(Turret.WantedState.IDLE)));
        tab.add("moveToPosition", new InstantCommand(() -> Turret.getInstance().setWantedState(Turret.WantedState.MOVE_TO_POSITION, targetAngle.getAsDouble())));

        //IS ON TARGET
        tab.addBoolean("is turret on target", () -> Turret.getInstance().isTurretOnTarget(TURRET_TOLERANCE));
    }
}
