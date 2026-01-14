package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.PID.PIDValues;

import static frc.robot.subsystems.turret.TurretConstants.TURRET_FORWARD_SOFT_LIMIT_THRESHOLD;
import static frc.robot.subsystems.turret.TurretConstants.TURRET_REVERSE_SOFT_LIMIT_THRESHOLD;

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

    private WantedState wantedState = WantedState.IDLE;
    private SystemState previousSystemState = SystemState.IDLE;
    private SystemState systemState = SystemState.IDLE;

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
    }

    @Override
    public void periodic() {
        turretIO.updateInputs(turretInputs);

        systemState = handleStateTransition();

        applyStates();

        log();
    }

    private void log() {
        turretInputs.turretMotorInputs.log();
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case IDLE -> SystemState.IDLE;
            case MOVE_TO_POSITION -> SystemState.MOVE_TO_POSITION;
        };
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
        double finalAngle = calculateMovingAngle(wantedAngle,new Rotation2d(getTurretAngle()),TURRET_FORWARD_SOFT_LIMIT_THRESHOLD,TURRET_REVERSE_SOFT_LIMIT_THRESHOLD);
        turretIO.moveToAngle(finalAngle);
    }

    private double calculateMovingAngle(double wantedAngle, Rotation2d currentAngle, double forwardLimit, double reverseLimit) {
        double currentTotalAngle = Units.rotationsToDegrees(currentAngle.getRotations());
        double closestOffset = wantedAngle - currentAngle.getDegrees();
        if (closestOffset > 180){
            closestOffset -= 360;
        }
        if (closestOffset < -180){
            closestOffset += 360;
        }

        double finalOffset = currentTotalAngle + closestOffset;
        if ((currentTotalAngle + closestOffset)%360 == (currentTotalAngle - closestOffset)%360){
            if (finalOffset > 0){
                finalOffset = currentTotalAngle - Math.abs(closestOffset);
            }
            else{
                finalOffset = currentTotalAngle + Math.abs(closestOffset);
            }
        }

        if (finalOffset > Units.degreesToRadians(forwardLimit)) {
            finalOffset -= 360;
        } else if (finalOffset < Units.degreesToRadians(reverseLimit)) {
            finalOffset += 360;
        }

        return finalOffset;
    }

    public boolean isTurretOnTarget(double turretTolerance) {
        return Math.abs(turretInputs.turretMotorInputs.getMotorValue().getAsDouble() - wantedAngle) < turretTolerance;
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
