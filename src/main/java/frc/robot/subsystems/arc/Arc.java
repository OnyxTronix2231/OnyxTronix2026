package frc.robot.subsystems.arc;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.PID.PIDValues;
import org.littletonrobotics.junction.Logger;

public class Arc extends SubsystemBase {
    private final ArcIO.ArcInputs arcInputs;
    private final ArcIO arcIO;

    public enum WantedState {
        IDLE,
        MOVE_TO_ANGLE,
        MOVE_FORWARDS
    }

    public enum SystemState {
        IDLING,
        MOVING_TO_ANGLE,
        MOVING_FORWARDS
    }

    private WantedState wantedState;
    private SystemState systemState;
    private SystemState previousSystemState;

    private double wantedAngle;

    public WantedState getWantedState() {
        return wantedState;
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;

    }

    public void setWantedState(WantedState wantedState, double wantedAngle) {
        this.wantedState = wantedState;

        this.wantedAngle = wantedAngle;
    }

    public SystemState getSystemState() {
        return systemState;
    }

    public Arc(ArcIO arcIO) {
        this.arcInputs = new ArcIO.ArcInputs();
        this.arcIO = arcIO;
        this.arcIO.updateInputs(arcInputs);

        this.wantedState = WantedState.IDLE;
        this.systemState = SystemState.IDLING;
        this.previousSystemState = SystemState.IDLING;

        this.wantedAngle = 0;
    }

    @Override
    public void periodic() {
        arcIO.updateInputs(arcInputs);

        double timestamp = Timer.getFPGATimestamp();

        systemState = handleStateTransition();

        applyStates();

        arcInputs.arcMotorInputs.log();

        previousSystemState = systemState;

        Logger.recordOutput("Subsystems/Arc/PeriodicTime", timestamp - Timer.getFPGATimestamp());
    }

    public SystemState handleStateTransition() {
        switch (wantedState) {
            case IDLE:
                return SystemState.IDLING;
            case MOVE_TO_ANGLE:
                return SystemState.MOVING_TO_ANGLE;
            case MOVE_FORWARDS:
                return SystemState.MOVING_FORWARDS;
        }
        return SystemState.IDLING;
    }

    public void applyStates() {
        switch (systemState) {
            case IDLING -> idleState();
            case MOVING_TO_ANGLE -> moveToAngleState();
            case MOVING_FORWARDS -> arcIO.setDutyCycle(0.05);
        }
    }

    private void idleState() {
        arcIO.setDutyCycle(0);
    }

    private void moveToAngleState() {
        arcIO.moveToAngle(wantedAngle);
    }

    public void updatePID(PIDValues pidValues) {
        arcIO.updatePID(pidValues);
    }

    public double getAngle() {
        return arcInputs.arcMotorInputs.getMotorValue().getAsDouble();
    }

    public double getVelocity() {
        return arcInputs.arcMotorInputs.getMotorAngularVelocityRotPerSec();
    }

    public double getAcceleration() {
        return arcInputs.arcMotorInputs.getMotorAngularAccelerationRadPerSecSquared();
    }

    private static Arc instance;

    public static void init(ArcIO arcIO) {
        if (instance == null) {
            instance = new Arc(arcIO);
        }
    }

    public static Arc getInstance() {
        return instance;
    }
}
