package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.PID.PIDValues;

import static frc.robot.Constants.DEGREES_IN_A_CIRCLE;
import static frc.robot.subsystems.turret.TurretConstants.*;

public class Turret extends SubsystemBase {
    private final TurretIO.TurretInputs turretInputs;
    private final TurretIO turretIO;

    private double wantedAngle;

    public enum WantedState {
        IDLE,
        MOVE_TO_POSITION,
    }

    public enum SystemState {
        IDLE,
        MOVE_TO_POSITION,
    }

    private WantedState wantedState;
    private SystemState previousSystemState;
    private SystemState systemState;

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public void setWantedState(WantedState wantedState, double wantedAngle) {
        this.wantedState = wantedState;
        this.wantedAngle = wantedAngle;
    }

    public WantedState getWantedState() {
        return wantedState;
    }

    public SystemState getSystemState() {
        return systemState;
    }

    public double getTurretAngle() {
        return turretInputs.turretMotorInputs.getMotorValue().getAsDouble();
    }

    public double getTurretVelocity() {
        return turretInputs.turretMotorInputs.getMotorAngularVelocityRotPerSec();
    }

    public double getTurretAcceleration() {
        return turretInputs.turretMotorInputs.getMotorAngularAccelerationRotPerSecSquared();
    }

    public Turret(TurretIO turretIO) {
        this.turretIO = turretIO;
        this.turretInputs = new TurretIO.TurretInputs();
        this.turretIO.updateInputs(turretInputs);

        wantedAngle = 0;
        wantedState = WantedState.IDLE;
        systemState = SystemState.IDLE;
        previousSystemState = SystemState.IDLE;
    }

    @Override
    public void periodic() {
        turretIO.updateInputs(turretInputs);

        systemState = handleStateTransition();

        applyStates();

        previousSystemState = systemState;

        log();
    }

    private void log() {
        turretInputs.turretMotorInputs.log();
    }

    private SystemState handleStateTransition() {
        switch (wantedState) {
            case IDLE:
                return SystemState.IDLE;
            case MOVE_TO_POSITION:
                return SystemState.MOVE_TO_POSITION;
        }
        return SystemState.IDLE;
    }

    private void applyStates() {
        switch (systemState) {
            case IDLE -> idleState();
            case MOVE_TO_POSITION -> moveToPosition();
        }
    }

    private void idleState() {
        turretIO.setDutyCycle(0);
    }

    private void moveToPosition() {
        wantedAngle = calculateMovingAngle(wantedAngle, getTurretAngle(), TURRET_FORWARD_SOFT_LIMIT_THRESHOLD, TURRET_REVERSE_SOFT_LIMIT_THRESHOLD);
        turretIO.moveToAngle(wantedAngle);
    }

    private double calculateMovingAngle(double wantedAngle, double currentAngle, double forwardLimit, double reverseLimit) {
        double closestOffset = wantedAngle - wrapTo360(currentAngle);

        if (closestOffset > DEGREES_IN_A_CIRCLE / 2) {
            closestOffset -= DEGREES_IN_A_CIRCLE;
        }
        if (closestOffset < -DEGREES_IN_A_CIRCLE / 2) {
            closestOffset += DEGREES_IN_A_CIRCLE;
        }

        double finalOffset = currentAngle + closestOffset;
        if (Math.abs(closestOffset - DEGREES_IN_A_CIRCLE / 2) < SAME_DISTANCE_THRESHOLD) {
            if (finalOffset > 0) {
                finalOffset = currentAngle - Math.abs(closestOffset);
            } else {
                finalOffset = currentAngle + Math.abs(closestOffset);
            }
        }

        if (finalOffset > forwardLimit) {
            finalOffset -= DEGREES_IN_A_CIRCLE;
        } else if (finalOffset < reverseLimit) {
            finalOffset += DEGREES_IN_A_CIRCLE;
        }
        return finalOffset;
    }

    private double wrapTo360(double angle) {
        return MathUtil.inputModulus(angle, 0, DEGREES_IN_A_CIRCLE);
    }


    public boolean isTurretOnTarget(double turretTolerance) {
        return Math.abs(wrapTo360(getTurretAngle()) - wrapTo360(wantedAngle)) < turretTolerance;
    }

    public void updateTurretPID(PIDValues pidValues) {
        turretIO.updatePID(pidValues);
    }

    private static Turret instance;

    public static void init(TurretIO turretIO) {
        if (instance == null) {
            instance = new Turret(turretIO);
        }
    }

    public static Turret getInstance() {
        return instance;
    }

}
