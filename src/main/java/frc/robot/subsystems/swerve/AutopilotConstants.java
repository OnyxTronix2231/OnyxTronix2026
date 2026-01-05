package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.lib.SqrtErrorProfiledPIDController;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.subsystems.swerve.generated.OffSeasonTunerConstants.kSpeedAt12Volts;

public class AutopilotConstants {
    //public static final double MAX_AUTOPILOT_SPEED = kSpeedAt12Volts.in(MetersPerSecond);
    public static double MAX_RELATIVE_AUTOPILOT_VELOCITY_X = 3.5;
    public static double MAX_RELATIVE_AUTOPILOT_VELOCITY_Y = 3.5;
    public static double MAX_RELATIVE_AUTOPILOT_VELOCITY_ROT = 3;

    public static final SqrtErrorProfiledPIDController RELATIVE_AUTOPILOT_X_CONTROLLER = new SqrtErrorProfiledPIDController(2.5, 0, 0.02, new TrapezoidProfile.Constraints(MAX_RELATIVE_AUTOPILOT_VELOCITY_X, kSpeedAt12Volts.in(MetersPerSecond)));
    public static final SqrtErrorProfiledPIDController RELATIVE_AUTOPILOT_Y_CONTROLLER = new SqrtErrorProfiledPIDController(2, 0, 0.02, new TrapezoidProfile.Constraints(MAX_RELATIVE_AUTOPILOT_VELOCITY_Y, kSpeedAt12Volts.in(MetersPerSecond)));
    public static final PIDController RELATIVE_AUTOPILOT_ROTATIONAL_CONTROLLER = new PIDController(4, 0, 0.03);

    public static final PIDController RELATIVE_AUTO_ASSIST_CORAL_Y_CONTROLLER = new PIDController(3, 0, 0.0);

    public static double MAX_RELATIVE_AUTOPILOT_VELOCITY_X_L4 = 2;
    public static double MAX_RELATIVE_AUTOPILOT_VELOCITY_Y_L4 = 2.5;

    public static final SqrtErrorProfiledPIDController RELATIVE_AUTOPILOT_X_CONTROLLER_L4 = new SqrtErrorProfiledPIDController(1.8, 0, 0.02, new TrapezoidProfile.Constraints(MAX_RELATIVE_AUTOPILOT_VELOCITY_X_L4, 0.5 * kSpeedAt12Volts.in(MetersPerSecond)));
    public static final SqrtErrorProfiledPIDController RELATIVE_AUTOPILOT_Y_CONTROLLER_L4 = new SqrtErrorProfiledPIDController(1.8, 0, 0.02, new TrapezoidProfile.Constraints(MAX_RELATIVE_AUTOPILOT_VELOCITY_Y_L4, 0.5 * kSpeedAt12Volts.in(MetersPerSecond)));

    static {
        RELATIVE_AUTOPILOT_X_CONTROLLER.setTolerance(0.03, 0);
        RELATIVE_AUTOPILOT_Y_CONTROLLER.setTolerance(0.03, 0);
        RELATIVE_AUTOPILOT_X_CONTROLLER_L4.setTolerance(0.025, 0);
        RELATIVE_AUTOPILOT_Y_CONTROLLER_L4.setTolerance(0.025, 0);
        RELATIVE_AUTOPILOT_ROTATIONAL_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
        RELATIVE_AUTOPILOT_ROTATIONAL_CONTROLLER.setTolerance(Rotation2d.fromDegrees(3).getRadians());
    }

    public static void update_RELATIVE_AUTOPILOT_Y_CONTROLLER_PID(double p, double i, double d) {
        RELATIVE_AUTOPILOT_Y_CONTROLLER.setPID(p, i, d);
    }

    public static void update_RELATIVE_AUTOPILOT_X_CONTROLLER_PID(double p, double i, double d) {
        RELATIVE_AUTOPILOT_X_CONTROLLER.setPID(p, i, d);
    }

    public static void update_RELATIVE_AUTOPILOT_ROTATIONAL_CONTROLLER_PID(double p, double i, double d) {
        RELATIVE_AUTOPILOT_ROTATIONAL_CONTROLLER.setPID(p, i, d);
    }

    public static void setMAX_RELATIVE_AUTOPILOT_VELOCITY_X(double speed) {
        MAX_RELATIVE_AUTOPILOT_VELOCITY_X = speed;
    }

    public static void setMAX_RELATIVE_AUTOPILOT_VELOCITY_Y(double speed) {
        MAX_RELATIVE_AUTOPILOT_VELOCITY_Y = speed;
    }

    public static void setMAX_RELATIVE_AUTOPILOT_VELOCITY_ROT(double speed) {
        MAX_RELATIVE_AUTOPILOT_VELOCITY_ROT = speed;
    }

    public static double rotErr() {
        return RELATIVE_AUTOPILOT_ROTATIONAL_CONTROLLER.getError();
    }

    public static double xErr() {
        return RELATIVE_AUTOPILOT_X_CONTROLLER.getError();
    }

    public static double yErr() {
        return RELATIVE_AUTOPILOT_Y_CONTROLLER.getError();
    }



}
