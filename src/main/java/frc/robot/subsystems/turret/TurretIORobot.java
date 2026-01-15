package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.OnyxMotorInputs;
import frc.robot.lib.PID.PIDValues;

import static frc.robot.subsystems.turret.TurretConstants.*;
import static frc.robot.subsystems.turret.TurretConstants.ROTATIONS_TO_ANGLE;
import static frc.robot.subsystems.turret.TurretConstants.TURRET_MOTOR_NAME;

public class TurretIORobot implements TurretIO {

    private final TalonFX motor;

    private final OnyxMotorInputs turretMotorInputs;

    private final MotionMagicVoltage motionMagicVoltage;

    private final CANcoder encoder;

    public TurretIORobot() {
        motor = new TalonFX(TURRET_MOTOR_ID);

        turretMotorInputs = new OnyxMotorInputs(motor, TURRET_SUBSYSTEM_NAME, TURRET_MOTOR_NAME, ROTATIONS_TO_ANGLE);

        motor.getConfigurator().apply(getTalonFXConfiguration());

        motionMagicVoltage = new MotionMagicVoltage(0.0).withSlot(TURRET_MOTION_MAGIC_DEFAULT_SLOT);

        encoder = new CANcoder(TURRET_CANCODER_ID);

        encoder.getConfigurator().apply(getMagnetSensorConfigs());
    }

    public TalonFXConfiguration getTalonFXConfiguration() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.Slot0 = TURRET_PID_VALUES.pidValuesToSlot0Configs();

        configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        configuration.MotionMagic.MotionMagicCruiseVelocity = TURRET_CRUISE_VELOCITY;
        configuration.MotionMagic.MotionMagicAcceleration = TURRET_ACCELERATION;
        configuration.MotionMagic.MotionMagicJerk = TURRET_JERK;

        configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(TURRET_FORWARD_SOFT_LIMIT_THRESHOLD);

        configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(TURRET_REVERSE_SOFT_LIMIT_THRESHOLD);

        configuration.HardwareLimitSwitch.ForwardLimitEnable = false;
        configuration.HardwareLimitSwitch.ReverseLimitEnable = false;

        configuration.CurrentLimits.StatorCurrentLimitEnable = true;
        configuration.CurrentLimits.StatorCurrentLimit = TURRET_STATOR_LIMIT;

        configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
        configuration.CurrentLimits.SupplyCurrentLimit = TURRET_SUPPLY_LIMIT;

        configuration.CurrentLimits.SupplyCurrentLowerLimit = TURRET_SUPPLY_LOWER_LIMIT;
        configuration.CurrentLimits.SupplyCurrentLowerTime = TURRET_TIME_LOWER_LIMIT;
        return configuration;
    }

    public MagnetSensorConfigs getMagnetSensorConfigs() {
        MagnetSensorConfigs configuration = new MagnetSensorConfigs();
        configuration.withMagnetOffset(TURRET_CANCODER_OFFSET);
        configuration.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
        return configuration;
    }

    @Override
    public void updateInputs(TurretInputs inputs) {
        turretMotorInputs.updateInputs();
        inputs.turretMotorInputs = turretMotorInputs;

        inputs.encoderPosition = encoder.getPosition().getValueAsDouble();
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        motor.set(dutyCycle);
    }

    @Override
    public void moveToAngle(double angle) {
        motor.setControl(motionMagicVoltage.withPosition(Units.degreesToRotations(angle)));
    }

    @Override
    public void updatePID(PIDValues pidValues) {
        pidValues.updatePIDValues(motor);
    }
}
