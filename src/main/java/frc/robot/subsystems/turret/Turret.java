package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.PID.PIDValues;

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
        return turretInputs.turretMotorInputs.getMotorAngularAccelerationRadPerSecSquared()   ;
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

    private void log(){
        turretInputs.turretMotorInputs.log();
    }

    private SystemState handleStateTransition(){
        return switch (wantedState) {
            case IDLE -> SystemState.IDLE;
            case MOVE_TO_POSITION -> SystemState.MOVE_TO_POSITION;
        };
    }

    private void applyStates(){
        switch(systemState){
            case IDLE -> idleState();
            case MOVE_TO_POSITION -> moveToPosition(wantedAngle);
        }
    }

    private void idleState(){
        turretIO.setDutyCycle(0);
    }

    private void moveToPosition(double angle){
        turretIO.moveToAngle(angle);
    }

    public boolean isTurretOnTarget(double turretTolerance){
        return Math.abs(turretInputs.turretMotorInputs.getMotorValue().getAsDouble()-wantedAngle) < turretTolerance;
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
