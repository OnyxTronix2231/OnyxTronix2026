package frc.robot.subsystems.hood;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import frc.robot.lib.PID.PIDValues;

import java.util.function.DoubleUnaryOperator;

public class HoodConstants {
    public static final String HOOD_SUBSYSTEM_NAME = "Hood";
    public static final String HOOD_MOTOR_NAME = "hoodMotor";

    public static final int HOOD_MOTOR_ID = 9;

    public static final int HOOD_CANCODER_ID = 8;
    public static final double HOOD_CANCODER_OFFSET = 0;

    public static final double HOOD_FORWARD_SOFT_LIMIT_DEGREES = 90;
    public static final double HOOD_REVERSE_SOFT_LIMIT_DEGREES = 0;

    public static final DoubleUnaryOperator ROTATIONS_TO_ANGLE = rotations -> rotations * 360;

    public static final double HOOD_KP = 0;
    public static final double HOOD_KI = 0;
    public static final double HOOD_KD = 0;
    public static final double HOOD_KG = 0;
    public static final double HOOD_KS = 0;
    public static final double HOOD_KV = 0;
    public static final double HOOD_KA = 0;
    public static final PIDValues HOOD_PID_VALUES = new PIDValues(
        HOOD_KP,
        HOOD_KI,
        HOOD_KD,
        HOOD_KG,
        HOOD_KS,
        HOOD_KV,
        HOOD_KA,
        GravityTypeValue.Arm_Cosine,
        StaticFeedforwardSignValue.UseVelocitySign
    );

    public static final double HOOD_CRUISE_VELOCITY = 10;
    public static final double HOOD_ACCELERATION = 10;
    public static final double HOOD_JERK = 0;


    public static final double SIMULATION_HOOD_KP = 5;
    public static final double SIMULATION_HOOD_KI = 0;
    public static final double SIMULATION_HOOD_KD = 0;
    public static final double SIMULATION_HOOD_KG = 0;
    public static final double SIMULATION_HOOD_KS = 0;
    public static final double SIMULATION_HOOD_KV = 0;
    public static final double SIMULATION_HOOD_KA = 0;
    public static final PIDValues SIMULATION_HOOD_PID_VALUES = new PIDValues(
        SIMULATION_HOOD_KP,
        SIMULATION_HOOD_KI,
        SIMULATION_HOOD_KD,
        SIMULATION_HOOD_KG,
        SIMULATION_HOOD_KS,
        SIMULATION_HOOD_KV,
        SIMULATION_HOOD_KA,
        GravityTypeValue.Arm_Cosine,
        StaticFeedforwardSignValue.UseVelocitySign
    );

    public static final double SIMULATION_HOOD_CRUISE_VELOCITY = 10;
    public static final double SIMULATION_HOOD_ACCELERATION = 10;
    public static final double SIMULATION_HOOD_JERK = 0;

    public static final double SIMULATION_HOOD_LENGTH_METERS = 0.001;
    public static final double SIMULATION_HOOD_MASS_KG = 0.001;
    public static final int SIMULATION_HOOD_NUM_OF_MOTORS = 1;

}
