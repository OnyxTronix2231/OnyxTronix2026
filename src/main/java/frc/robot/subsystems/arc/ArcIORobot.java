package frc.robot.subsystems.arc;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.OnyxMotorInputs;
import frc.robot.lib.PID.PIDValues;

import static frc.robot.subsystems.arc.ArcConstants.*;

public class ArcIORobot implements ArcIO {
    private final TalonFX motor;

    private final OnyxMotorInputs arcMotorInputs;

    private final MotionMagicVoltage motionMagicVoltage;

    public ArcIORobot() {
        motor = new TalonFX(ARC_MOTOR_ID);

        arcMotorInputs = new OnyxMotorInputs(motor, ARC_SUBSYSTEM_NAME, ARC_MOTOR_NAME, ROTATIONS_TO_ANGLE);

        motor.getConfigurator().apply(getTalonFXConfiguration());

        motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
    }

    public TalonFXConfiguration getTalonFXConfiguration() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.Slot0 = ARC_PID_VALUES.pidValuesToSlot0Configs();

        configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        configuration.MotionMagic.MotionMagicCruiseVelocity = ARC_CRUISE_VELOCITY;
        configuration.MotionMagic.MotionMagicAcceleration = ARC_ACCELERATION;
        configuration.MotionMagic.MotionMagicJerk = ARC_JERK;

        configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(ARC_FORWARD_SOFT_LIMIT_DEGREES);

        configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(ARC_REVERSE_SOFT_LIMIT_DEGREES);

        return configuration;
    }

    @Override
    public void updateInputs(ArcInputs inputs) {
        arcMotorInputs.updateInputs();
        inputs.arcMotorInputs = arcMotorInputs;
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
