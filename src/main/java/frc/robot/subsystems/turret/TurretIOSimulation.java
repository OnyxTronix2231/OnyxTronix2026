package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.lib.OnyxMotorInputs;
import frc.robot.lib.PID.PIDValues;

import static frc.robot.Constants.SIMULATION_DT_SECONDS;
import static frc.robot.subsystems.turret.TurretConstants.*;

public class TurretIOSimulation implements TurretIO {

    private final TalonFX motor;
    private final DCMotorSim simulatedMotor;

    private final OnyxMotorInputs turretMotorInputs;

    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0).withSlot(TURRET_MOTION_MAGIC_DEFAULT_SLOT);

    public TurretIOSimulation() {
        motor = new TalonFX(TURRET_MOTOR_ID);
        simulatedMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(SIMULATION_TURRET_NUM_OF_MOTORS),
                SingleJointedArmSim.estimateMOI(SIMULATION_TURRET_LENGTH_METERS, SIMULATION_TURRET_MASS_KG), TURRET_ROTOR_TO_SENSOR_RATIO),
                DCMotor.getKrakenX60(SIMULATION_TURRET_NUM_OF_MOTORS));
        turretMotorInputs = new OnyxMotorInputs(motor,TURRET_SUBSYSTEM_NAME,TURRET_MOTOR_NAME,ROTATIONS_TO_ANGLE);
        motor.getConfigurator().apply(getTalonFXConfiguration());
        
    }

    public TalonFXConfiguration getTalonFXConfiguration() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.Slot0 = SIMULATION_TURRET_PID_VALUES.pidValuesToSlot0Configs();

        configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        configuration.MotionMagic.MotionMagicCruiseVelocity = SIMULATION_TURRET_CRUISE_VELOCITY;
        configuration.MotionMagic.MotionMagicAcceleration = SIMULATION_TURRET_ACCELERATION;
        configuration.MotionMagic.MotionMagicJerk = SIMULATION_TURRET_JERK;

        configuration.CurrentLimits.StatorCurrentLimitEnable = true;
        configuration.CurrentLimits.StatorCurrentLimit = TURRET_STATOR_LIMIT;

        configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
        configuration.CurrentLimits.SupplyCurrentLimit = TURRET_SUPPLY_LIMIT;

        configuration.CurrentLimits.SupplyCurrentLowerLimit = TURRET_SUPPLY_LOWER_LIMIT;
        configuration.CurrentLimits.SupplyCurrentLowerTime = TURRET_TIME_LOWER_LIMIT;
        return configuration;
    }

    @Override
    public void updateInputs(TurretInputs inputs) {
        updateMotor();
        turretMotorInputs.updateInputs();
        inputs.turretMotorInputs = turretMotorInputs;

        inputs.encoderPosition = 0;
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
