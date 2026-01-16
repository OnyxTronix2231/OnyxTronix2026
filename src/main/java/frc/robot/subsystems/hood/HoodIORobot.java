package frc.robot.subsystems.hood;

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

import static frc.robot.subsystems.hood.HoodConstants.*;

public class HoodIORobot implements HoodIO {

    private final TalonFX motor;

    private final OnyxMotorInputs hoodMotorInputs;

    private final MotionMagicVoltage motionMagicVoltage;

    private final CANcoder encoder;

    public HoodIORobot() {
        motor = new TalonFX(HOOD_MOTOR_ID);

        hoodMotorInputs = new OnyxMotorInputs(motor, HOOD_SUBSYSTEM_NAME, HOOD_MOTOR_NAME, ROTATIONS_TO_ANGLE);

        motor.getConfigurator().apply(getTalonFXConfiguration());

        motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);

        encoder = new CANcoder(HOOD_CANCODER_ID);

        encoder.getConfigurator().apply(getMagnetSensorConfigs());
    }

    public TalonFXConfiguration getTalonFXConfiguration() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.Slot0 = HOOD_PID_VALUES.pidValuesToSlot0Configs();

        configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        configuration.MotionMagic.MotionMagicCruiseVelocity = HOOD_CRUISE_VELOCITY;
        configuration.MotionMagic.MotionMagicAcceleration = HOOD_ACCELERATION;
        configuration.MotionMagic.MotionMagicJerk = HOOD_JERK;

        configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(HOOD_FORWARD_SOFT_LIMIT_DEGREES);

        configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(HOOD_REVERSE_SOFT_LIMIT_DEGREES);

        return configuration;
    }

    public MagnetSensorConfigs getMagnetSensorConfigs() {
        MagnetSensorConfigs configs = new MagnetSensorConfigs();

        configs.withMagnetOffset(HOOD_CANCODER_OFFSET);

        configs.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

        return configs;
    }

    @Override
    public void updateInputs(HoodInputs inputs) {
        hoodMotorInputs.updateInputs();
        inputs.hoodMotorInputs = hoodMotorInputs;

        inputs.cancoderPosition = encoder.getPosition().getValueAsDouble();
    }

    @Override
    public void updatePID(PIDValues pidValues) {
        pidValues.updatePIDValues(motor);
    }

    @Override
    public void moveToAngle(double angle) {
        motor.setControl(motionMagicVoltage.withPosition(Units.degreesToRotations(angle)));
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        motor.set(dutyCycle);
    }
}
