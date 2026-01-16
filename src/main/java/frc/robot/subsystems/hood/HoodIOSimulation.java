package frc.robot.subsystems.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.lib.OnyxMotorInputs;
import frc.robot.lib.PID.PIDValues;

import static frc.robot.Constants.SIMULATION_DT_SECONDS;
import static frc.robot.subsystems.hood.HoodConstants.*;

public class HoodIOSimulation implements HoodIO {

    private final TalonFX motor;
    private final DCMotorSim simulatedMotor;

    private final OnyxMotorInputs hoodMotorInputs;

    private final MotionMagicVoltage motionMagicVoltage;

    public HoodIOSimulation() {
        motor = new TalonFX(HOOD_MOTOR_ID);
        simulatedMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(SIMULATION_HOOD_NUM_OF_MOTORS),
            SingleJointedArmSim.estimateMOI(SIMULATION_HOOD_LENGTH_METERS, SIMULATION_HOOD_MASS_KG), 1),
            DCMotor.getKrakenX60(SIMULATION_HOOD_NUM_OF_MOTORS));

        hoodMotorInputs = new OnyxMotorInputs(motor, HOOD_SUBSYSTEM_NAME, HOOD_MOTOR_NAME, ROTATIONS_TO_ANGLE);

        motor.getConfigurator().apply(getTalonFXConfiguration());

        motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
    }

    public TalonFXConfiguration getTalonFXConfiguration() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.Slot0 = SIMULATION_HOOD_PID_VALUES.pidValuesToSlot0Configs();

        configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        configuration.MotionMagic.MotionMagicCruiseVelocity = SIMULATION_HOOD_CRUISE_VELOCITY;
        configuration.MotionMagic.MotionMagicAcceleration = SIMULATION_HOOD_ACCELERATION;
        configuration.MotionMagic.MotionMagicJerk = SIMULATION_HOOD_JERK;

        configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(HOOD_FORWARD_SOFT_LIMIT_DEGREES);

        configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(HOOD_REVERSE_SOFT_LIMIT_DEGREES);

        return configuration;
    }

    @Override
    public void updateInputs(HoodInputs inputs) {
        updateMotor();

        hoodMotorInputs.updateInputs();
        inputs.hoodMotorInputs = hoodMotorInputs;
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

    public void updateMotor() {
        TalonFXSimState motorSimState = motor.getSimState();
        motor.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
        simulatedMotor.setInputVoltage(motorSimState.getMotorVoltage());
        simulatedMotor.update(SIMULATION_DT_SECONDS);

        motorSimState.setRawRotorPosition(simulatedMotor.getAngularPositionRotations());
        motorSimState.setRotorVelocity(
            Units.radiansToRotations(simulatedMotor.getAngularVelocityRadPerSec()));
    }
}
