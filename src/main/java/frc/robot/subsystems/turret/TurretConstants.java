package frc.robot.subsystems.turret;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import frc.robot.lib.PID.PIDValues;

import java.util.function.DoubleUnaryOperator;

public class TurretConstants {
    public static final String TURRET_SUBSYSTEM_NAME = "Turret";
    public static final String TURRET_MOTOR_NAME = "turretMotor";

    public static final int TURRET_MOTOR_ID = 1;

    public static final double TURRET_STATOR_LIMIT = 120;
    public static final double TURRET_SUPPLY_LIMIT = 70;
    public static final double TURRET_SUPPLY_LOWER_LIMIT = 40;
    public static final double TURRET_TIME_LOWER_LIMIT = 1;


    public static final double TURRET_ROTOR_TO_SENSOR_RATIO = 1;
    public static final int TURRET_MOTION_MAGIC_DEFAULT_SLOT = 0;

    public static final DoubleUnaryOperator ROTATIONS_TO_ANGLE = rotations -> rotations * 360;

    public static final double TURRET_TOLERANCE = 2;

    public static final double TURRET_KP = 10;
    public static final double TURRET_KI = 0;
    public static final double TURRET_KD = 0;
    public static final double TURRET_KG = 0;
    public static final double TURRET_KS = 0;
    public static final double TURRET_KV = 0;
    public static final double TURRET_KA = 0;
    public static final PIDValues TURRET_PID_VALUES = new PIDValues(
            TURRET_KP,
            TURRET_KI,
            TURRET_KD,
            TURRET_KG,
            TURRET_KS,
            TURRET_KV,
            TURRET_KA,
            GravityTypeValue.Arm_Cosine,
            StaticFeedforwardSignValue.UseVelocitySign);


    //SIMULATION
    public static final double SIMULATION_TURRET_CRUISE_VELOCITY = 3;
    public static final double SIMULATION_TURRET_ACCELERATION = 5.5;
    public static final double SIMULATION_TURRET_JERK = 0;

    public static final double SIMULATION_TURRET_KP = 10;
    public static final double SIMULATION_TURRET_KI = 0;
    public static final double SIMULATION_TURRET_KD = 0;
    public static final double SIMULATION_TURRET_KG = 0;
    public static final double SIMULATION_TURRET_KS = 0;
    public static final double SIMULATION_TURRET_KV = 0;
    public static final double SIMULATION_TURRET_KA = 0;
    public static final PIDValues SIMULATION_TURRET_PID_VALUES = new PIDValues(
            SIMULATION_TURRET_KP,
            SIMULATION_TURRET_KI,
            SIMULATION_TURRET_KD,
            SIMULATION_TURRET_KG,
            SIMULATION_TURRET_KS,
            SIMULATION_TURRET_KV,
            SIMULATION_TURRET_KA,
            GravityTypeValue.Arm_Cosine,
            StaticFeedforwardSignValue.UseVelocitySign);

    public static final double SIMULATION_TURRET_LENGTH_METERS = 0.001;
    public static final double SIMULATION_TURRET_MASS_KG = 0.001;
    public static final int SIMULATION_TURRET_NUM_OF_MOTORS = 1;
}
