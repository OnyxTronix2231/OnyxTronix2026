package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.PID.PIDValues;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
    private final HoodIO.HoodInputs hoodInputs;
    private final HoodIO hoodIO;

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

    public Hood(HoodIO hoodIO) {
        this.hoodInputs = new HoodIO.HoodInputs();
        this.hoodIO = hoodIO;
        this.hoodIO.updateInputs(hoodInputs);

        this.wantedState = WantedState.IDLE;
        this.systemState = SystemState.IDLING;
        this.previousSystemState = SystemState.IDLING;

        this.wantedAngle = 0;
    }

    @Override
    public void periodic() {
        hoodIO.updateInputs(hoodInputs);

        double timestamp = Timer.getFPGATimestamp();

        systemState = handleStateTransition();

        applyStates();

        hoodInputs.hoodMotorInputs.log();

        previousSystemState = systemState;

        Logger.recordOutput("Subsystems/Hood/PeriodicTime", timestamp - Timer.getFPGATimestamp());
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
            case MOVING_FORWARDS -> hoodIO.setDutyCycle(0.05);
        }
    }

    private void idleState() {
        hoodIO.setDutyCycle(0);
    }

    private void moveToAngleState() {
        hoodIO.moveToAngle(wantedAngle);
    }

    public void updatePID(PIDValues pidValues) {
        hoodIO.updatePID(pidValues);
    }

    public double getAngle() {
        return hoodInputs.hoodMotorInputs.getMotorValue().getAsDouble();
    }

    public double getVelocity() {
        return hoodInputs.hoodMotorInputs.getMotorAngularVelocityRotPerSec();
    }

    public double getAcceleration() {
        return hoodInputs.hoodMotorInputs.getMotorAngularAccelerationRadPerSecSquared();
    }

    private static Hood instance;

    public static void init(HoodIO hoodIO) {
        if (instance == null) {
            instance = new Hood(hoodIO);
        }
    }

    public static Hood getInstance() {
        return instance;
    }
}
