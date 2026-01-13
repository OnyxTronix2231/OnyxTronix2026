package frc.robot.lib;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;

public class OnyxMotorInputs {

    private final int motorID;
    private final StatusSignal<Temperature> motorTemperatureSignal;
    private final StatusSignal<AngularVelocity> motorAngularVelocityRadPerSecSignal;
    private final StatusSignal<AngularAcceleration> motorAngularAccelerationRadPerSecSquaredSignal;
    private final StatusSignal<Current> motorStatorCurrentAmpsSignal;
    private final StatusSignal<Voltage> motorAppliedVoltsSignal;
    private final StatusSignal<Current> motorSupplyCurrentAmpsSignal;
    private final StatusSignal<Angle> motorRawValueSignal;
    private DoubleSupplier motorValue;
    private final String motorName;
    private final DoubleUnaryOperator gearingFunction;
    private final String logBase;

    private double motorTemperature;
    private double motorAngularVelocityRadPerSec;
    private double motorAngularAccelerationRadPerSecSquared;
    private double motorStatorCurrentAmps;
    private double motorAppliedVolts;
    private double motorSupplyCurrentAmps;
    private double motorRawValue;

    public OnyxMotorInputs(TalonFX motor, String subsystemName, String motorName, DoubleUnaryOperator gearingFunction) {
        this.gearingFunction = gearingFunction;
        motorID = motor.getDeviceID();
        motorTemperatureSignal = motor.getDeviceTemp();
        motorAngularVelocityRadPerSecSignal = motor.getVelocity();
        motorAngularAccelerationRadPerSecSquaredSignal = motor.getAcceleration();
        motorStatorCurrentAmpsSignal = motor.getStatorCurrent();
        motorAppliedVoltsSignal = motor.getMotorVoltage();
        motorSupplyCurrentAmpsSignal = motor.getSupplyCurrent();
        motorRawValueSignal = motor.getPosition();
        motorValue = () -> gearingFunction.applyAsDouble(motorRawValueSignal.getValueAsDouble());
        logBase = "Subsystems/" + subsystemName + "/";
        this.motorName = motorName;

        motorTemperature = motorTemperatureSignal.getValueAsDouble();
        motorAngularVelocityRadPerSec = motorAngularVelocityRadPerSecSignal.getValueAsDouble();
        motorAngularAccelerationRadPerSecSquared = motorAngularAccelerationRadPerSecSquaredSignal.getValueAsDouble();
        motorStatorCurrentAmps = motorStatorCurrentAmpsSignal.getValueAsDouble();
        motorAppliedVolts = motorAppliedVoltsSignal.getValueAsDouble();
        motorSupplyCurrentAmps = motorSupplyCurrentAmpsSignal.getValueAsDouble();
        motorRawValue = motorRawValueSignal.getValueAsDouble();
    }

    public OnyxMotorInputs(TalonFX motor, String subsystemName, String motorName) {
        this(motor, subsystemName, motorName, angle -> angle);
    }

    public OnyxMotorInputs() {
        motorID = 0;
        motorTemperature = 0;
        motorAngularVelocityRadPerSec = 0;
        motorAngularAccelerationRadPerSecSquared = 0;
        motorStatorCurrentAmps = 0;
        motorAppliedVolts = 0;
        motorSupplyCurrentAmps = 0;
        motorRawValue = 0;

        motorTemperatureSignal = null;
        motorAngularVelocityRadPerSecSignal = null;
        motorAngularAccelerationRadPerSecSquaredSignal = null;
        motorStatorCurrentAmpsSignal = null;
        motorAppliedVoltsSignal = null;
        motorSupplyCurrentAmpsSignal = null;
        motorRawValueSignal = null;
        motorName = "";
        logBase = "";
        gearingFunction = angle -> angle;
    }

    public void updateInputs() {
        BaseStatusSignal.refreshAll(
                motorTemperatureSignal,
                motorAngularVelocityRadPerSecSignal,
                motorAngularAccelerationRadPerSecSquaredSignal,
                motorStatorCurrentAmpsSignal,
                motorAppliedVoltsSignal,
                motorSupplyCurrentAmpsSignal,
                motorRawValueSignal
        );
        motorTemperature = motorTemperatureSignal.getValueAsDouble();
        motorAngularVelocityRadPerSec = motorAngularVelocityRadPerSecSignal.getValueAsDouble();
        motorAngularAccelerationRadPerSecSquared = motorAngularAccelerationRadPerSecSquaredSignal.getValueAsDouble();
        motorStatorCurrentAmps = motorStatorCurrentAmpsSignal.getValueAsDouble();
        motorAppliedVolts = motorAppliedVoltsSignal.getValueAsDouble();
        motorSupplyCurrentAmps = motorSupplyCurrentAmpsSignal.getValueAsDouble();
        motorRawValue = motorRawValueSignal.getValueAsDouble();
        motorValue = () -> gearingFunction.applyAsDouble(motorRawValueSignal.getValueAsDouble());
    }

    public void log() {
        Logger.recordOutput(logBase + motorName + "/RawValue", motorRawValueSignal.getValueAsDouble());
        Logger.recordOutput(logBase + motorName + "/ConvertedValue", motorValue);
        Logger.recordOutput(logBase + motorName + "/AppliedVolts", motorAppliedVoltsSignal.getValueAsDouble());
        Logger.recordOutput(logBase + motorName + "/SupplyCurrentAmps", motorSupplyCurrentAmpsSignal.getValueAsDouble());
        Logger.recordOutput(logBase + motorName + "/StatorCurrentAmps", motorStatorCurrentAmpsSignal.getValueAsDouble());
        Logger.recordOutput(logBase + motorName + "/AngularVelocityRadPerSec", motorAngularVelocityRadPerSecSignal.getValueAsDouble());
        Logger.recordOutput(logBase + motorName + "/AngularAccelerationRadPerSecSquared", motorAngularAccelerationRadPerSecSquaredSignal.getValueAsDouble());
    }

    public double getMotorAppliedVolts() {
        return motorAppliedVolts;
    }

    public double getMotorSupplyCurrentAmps() {
        return motorSupplyCurrentAmps;
    }

    public double getMotorStatorCurrentAmps() {
        return motorStatorCurrentAmps;
    }

    public double getMotorAngularVelocityRotPerSec() {
        return motorAngularVelocityRadPerSec;
    }

    public double getMotorAngularAccelerationRadPerSecSquared() {
        return motorAngularAccelerationRadPerSecSquared;
    }

    public double getMotorTemperature() {
        return motorTemperature;
    }

    public double getMotorRawValue() {
        return motorRawValue;
    }

    public int getMotorID() {
        return motorID;
    }

    public String getLogBase() {
        return logBase;
    }

    public String getMotorName() {
        return motorName;
    }

    public DoubleSupplier getMotorValue() {
        return motorValue;
    }
}