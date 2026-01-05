package frc.robot.lib;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SqrtErrorProfiledPIDController {
    // PID gains.
    private double kP, kI, kD;
    // WPILib motion profile constraints.
    private final TrapezoidProfile.Constraints constraints;

    // Goal and current setpoint as defined by the trapezoidal profile.
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setpoint;

    private double messure;

    // Tolerances to determine if the controller has reached its goal.
    private double posTolerance;
    private double velTolerance;

    // PID state.
    private double totalError;
    private double prevModifiedError;

    // Time tracking.
    private double lastTimestamp;

    /**
     * Constructor.
     *
     * @param kP          Proportional gain.
     * @param kI          Integral gain.
     * @param kD          Derivative gain.
     * @param constraints TrapezoidProfile constraints (max velocity and acceleration).
     */
    public SqrtErrorProfiledPIDController(double kP, double kI, double kD, TrapezoidProfile.Constraints constraints) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.constraints = constraints;

        // Initialize goal and setpoint to zero.
        this.goal = new TrapezoidProfile.State(0, 0);
        this.setpoint = new TrapezoidProfile.State(0, 0);

        // Default tolerances (can be adjusted via setTolerance()).
        this.posTolerance = 0.0;
        this.velTolerance = 0.0;

        this.totalError = 0;
        this.prevModifiedError = 0;
        this.lastTimestamp = System.currentTimeMillis() / 1000.0; // seconds
    }

    public void setPID(double kp, double ki, double kd) {
        this.kP = kp;
        this.kI = ki;
        this.kD = kd;
    }

    public double getError() {
        return Math.abs(goal.position - messure);
    }

    /**
     * Sets the goal position with zero velocity.
     *
     * @param goalPosition The desired final position.
     */
    public void setGoal(double goalPosition) {
        this.goal = new TrapezoidProfile.State(goalPosition, 0);
    }

    /**
     * Sets the goal state.
     *
     * @param goal The desired final state (position and velocity).
     */
    public void setGoal(TrapezoidProfile.State goal) {
        this.goal = goal;
    }

    /**
     * Returns the current goal state.
     *
     * @return The goal state.
     */
    public TrapezoidProfile.State getGoal() {
        return goal;
    }

    /**
     * Returns the current profiled setpoint.
     *
     * @return The setpoint state.
     */
    public TrapezoidProfile.State getSetpoint() {
        return setpoint;
    }

    /**
     * Sets the tolerances for position and velocity.
     *
     * @param posTolerance Allowed position error.
     * @param velTolerance Allowed velocity error.
     */
    public void setTolerance(double posTolerance, double velTolerance) {
        this.posTolerance = posTolerance;
        this.velTolerance = velTolerance;
    }

    /**
     * Resets the controller state.
     *
     * @param initialState The starting state (position and velocity).
     */
    public void reset(TrapezoidProfile.State initialState) {
        this.setpoint = new TrapezoidProfile.State(initialState.position, initialState.velocity);
        this.totalError = 0;
        this.prevModifiedError = 0;
        this.lastTimestamp = System.currentTimeMillis() / 1000.0;
    }

    /**
     * Returns true if the controller is at the goal within the defined tolerances.
     *
     * @return True if at goal, false otherwise.
     */
    public boolean atGoal() {
        return Math.abs(goal.position - setpoint.position) < posTolerance &&
                Math.abs(goal.velocity - setpoint.velocity) < velTolerance;
    }

    /**
     * Calculates the controller output based on the current measurement.
     * <p>
     * This method updates the motion profile using WPILib's TrapezoidProfile,
     * applies a square-root transform to the error, and then computes the PID output.
     *
     * @param measurement The current sensor measurement.
     * @return The computed output.
     */
    public double calculate(double measurement) {
        messure = measurement;
        double now = System.currentTimeMillis() / 1000.0;
        double dt = now - lastTimestamp;
        if (dt <= 0) {
            dt = 0.001;  // Prevent division by zero.
        }
        lastTimestamp = now;

        // --- Update the Motion Profile ---
        // Create a new trapezoidal profile with the current goal and setpoint.
        TrapezoidProfile profile = new TrapezoidProfile(constraints);
        // Calculate the next state based on the elapsed time.
        setpoint = profile.calculate(dt, setpoint, goal);

        // --- PID Calculation with Square-Root Error Transformation ---
        // Compute error between the profiled setpoint and the measurement.
        double error = setpoint.position - measurement;
        // Transform the error: take the square root of the absolute error while preserving its sign.
        double modifiedError = Math.signum(error) * Math.sqrt(Math.abs(error));

        // Update the integral and derivative terms.
        totalError += modifiedError * dt;
        double derivative = (modifiedError - prevModifiedError) / dt;
        prevModifiedError = modifiedError;

        // Compute the PID output.
        double pidOutput = kP * modifiedError + kI * totalError + kD * derivative;

        // Combine with the feed-forward term (setpoint velocity) for the final output.
        return Math.abs(error) < posTolerance ? 0 : (pidOutput + setpoint.velocity);
    }
}
