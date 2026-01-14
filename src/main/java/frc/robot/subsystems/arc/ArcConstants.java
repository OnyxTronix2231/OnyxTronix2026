package frc.robot.subsystems.arc;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import frc.robot.lib.PID.PIDValues;

import java.util.function.DoubleUnaryOperator;

public class ArcConstants {
    public static final int ARC_MOTOR_ID = 9;

    public static final int ARC_NUM_OF_MOTORS = 1;

    public static final DoubleUnaryOperator ROTATIONS_TO_DEGREES = rotations -> rotations * 360;
    public static final DoubleUnaryOperator DEGREES_TO_ROTATIONS = degrees -> degrees / 360;

    public static final double ARC_FORWARD_LIMIT_DEGREES = 90;
    public static final double ARC_REVERSE_LIMIT_DEGREES = 0;

    public static final double SIMULATION_ARC_KP = 5;
    public static final double SIMULATION_ARC_KI = 0;
    public static final double SIMULATION_ARC_KD = 0;
    public static final double SIMULATION_ARC_KG = 0;
    public static final double SIMULATION_ARC_KS = 0;
    public static final double SIMULATION_ARC_KV = 0;
    public static final double SIMULATION_ARC_KA = 0;
    public static final PIDValues SIMULATION_ARC_PID_VALUES = new PIDValues(
        SIMULATION_ARC_KP, SIMULATION_ARC_KI, SIMULATION_ARC_KD,
        SIMULATION_ARC_KG, SIMULATION_ARC_KS, SIMULATION_ARC_KV, SIMULATION_ARC_KA,
        GravityTypeValue.Arm_Cosine, StaticFeedforwardSignValue.UseVelocitySign
    );


    public static final double SIMULATION_ARC_LENGTH_METERS = 0.001;
    public static final double SIMULATION_ARC_MASS_KG = 0.001;

    public static final double SIMULATION_ARC_CRUISE_VELOCITY = 10;
    public static final double SIMULATION_ARC_ACCELERATION = 10;
    public static final double SIMULATION_ARC_JERK = 0;
}
