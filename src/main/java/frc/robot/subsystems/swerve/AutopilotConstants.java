package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.lib.SqrtErrorProfiledPIDController;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.subsystems.swerve.generated.OffSeasonTunerConstants.kSpeedAt12Volts;

public class AutopilotConstants {
    public static double MAX_RELATIVE_AUTOPILOT_VELOCITY_X = 3;
    public static double MAX_RELATIVE_AUTOPILOT_VELOCITY_Y = 3;
    public static double MAX_RELATIVE_AUTOPILOT_VELOCITY_ROT = 3;

    public static final SqrtErrorProfiledPIDController RELATIVE_AUTOPILOT_X_CONTROLLER = new SqrtErrorProfiledPIDController(1, 0, 0.0, new TrapezoidProfile.Constraints(MAX_RELATIVE_AUTOPILOT_VELOCITY_X, kSpeedAt12Volts.in(MetersPerSecond)));
    public static final SqrtErrorProfiledPIDController RELATIVE_AUTOPILOT_Y_CONTROLLER = new SqrtErrorProfiledPIDController(1, 0, 0.0, new TrapezoidProfile.Constraints(MAX_RELATIVE_AUTOPILOT_VELOCITY_Y, kSpeedAt12Volts.in(MetersPerSecond)));
    public static final PIDController RELATIVE_AUTOPILOT_ROTATIONAL_CONTROLLER = new PIDController(3, 0, 0.0);

}
