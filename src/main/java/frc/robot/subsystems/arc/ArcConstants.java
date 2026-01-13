package frc.robot.subsystems.arc;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import frc.robot.lib.PID.PIDValues;

public class ArcConstants {
    public static final int ARC_MOTOR_ID = 1;

    public static final int ARC_NUM_OF_MOTORS = 1;

    public static final int ARC_MOTION_MAGIC_DEFAULT_SLOT = 0;

    public static final double ARC_KP = 0;
    public static final double ARC_KI = 0;
    public static final double ARC_KD = 0;
    public static final double ARC_KG = 0;
    public static final double ARC_KS = 0;
    public static final double ARC_KV = 0;
    public static final double ARC_KA = 0;
    public static final PIDValues SIMULATION_ARC_PID_VALUES = new PIDValues(
        ARC_KP, ARC_KI, ARC_KD, ARC_KG, ARC_KS, ARC_KV, ARC_KA,
        GravityTypeValue.Arm_Cosine, StaticFeedforwardSignValue.UseVelocitySign
    );


    public static final int SIMULATION_ARC_LENGTH_METERS = 1;
    public static final int SIMULATION_ARC_MASS_KG = 1;

    public static final double SIMULATION_ARC_CRUISE_VELOCITY = 1;
    public static final double SIMULATION_ARC_ACCELERATION = 1;
    public static final double SIMULATION_ARC_JERK = 0;
}
