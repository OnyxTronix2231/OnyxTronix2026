package frc.robot.subsystems.arc;

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
import static frc.robot.subsystems.arc.ArcConstants.*;

public class ArcIOSimulation implements ArcIO {
    private final TalonFX motor;
    private final DCMotorSim simulatedMotor;

    private final OnyxMotorInputs arcMotorInputs;

    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);

    public ArcIOSimulation() {
        motor = new TalonFX(ARC_MOTOR_ID);
        simulatedMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(ARC_NUM_OF_MOTORS),
            SingleJointedArmSim.estimateMOI(SIMULATION_ARC_LENGTH_METERS, SIMULATION_ARC_MASS_KG), 1),
            DCMotor.getKrakenX60(ARC_NUM_OF_MOTORS));

        arcMotorInputs = new OnyxMotorInputs(motor, "Arc", "ArcMotor", ROTATIONS_TO_DEGREES);

        motor.getConfigurator().apply(getTalonFXConfiguration());
    }

    public TalonFXConfiguration getTalonFXConfiguration() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.Slot0 = SIMULATION_ARC_PID_VALUES.pidValuesToSlot0Configs();

        configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        configuration.MotionMagic.MotionMagicCruiseVelocity = SIMULATION_ARC_CRUISE_VELOCITY;
        configuration.MotionMagic.MotionMagicAcceleration = SIMULATION_ARC_ACCELERATION;
        configuration.MotionMagic.MotionMagicJerk = SIMULATION_ARC_JERK;

        return configuration;
    }

    @Override
    public void updateInputs(ArcInputs inputs) {
        updateMotor();

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
