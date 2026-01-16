package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.data.ScoringManager;
import frc.robot.lib.SqrtErrorProfiledPIDController;
import frc.robot.subsystems.alliance.AllianceProvider;
import frc.robot.subsystems.swerve.generated.OffSeasonTunerConstants;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static frc.robot.DriverOi.controller;
import static frc.robot.subsystems.localization.TargetVector.getRelativeOffsetFromFieldPoint;
import static frc.robot.subsystems.swerve.AutopilotConstants.*;
import static frc.robot.subsystems.swerve.generated.OffSeasonTunerConstants.kSpeedAt12Volts;
//import static frc.robot.Robot.hasUpdatedRotation;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends OffSeasonTunerConstants.TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private final double maxSpeed = kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;
    private double mJoystickLastTouched = -1;
    public static Optional<Rotation2d> mHeadingSetpoint = Optional.empty();
    public SlewRateLimiter limitX = new SlewRateLimiter(14);
    public SlewRateLimiter limitY = new SlewRateLimiter(14);


    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    public static final DoubleSupplier ALLIANCE_FLIPPER = () -> AllianceProvider.getInstance().isBlueAlliance() ? -1 : 1;


    public enum WantedState {
        IDLE,
        AUTO,
        TELEOP,
        AUTO_PILOT,
        AUTO_PILOT_CLIMBING
    }

    private enum SystemState {
        IDLE,
        AUTO,
        TELEOP,
        KEEP_HEADING,
        AUTO_PILOT,
        AUTO_PILOT_CLIMBING
    }

    private WantedState wantedState = WantedState.TELEOP;
    private SystemState currentSystemState = SystemState.TELEOP;
    private final SystemState previousSystemState = SystemState.TELEOP;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>c
     * This constructs the underlying hardware devices, so user should not construct
     * the devices themselves. If they need the devices, they can access them
     * through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants... modules) {
        super(drivetrainConstants, modules);
        configureAutoBuilder();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        CommandScheduler.getInstance().registerSubsystem(this);
        driveWithHeading.HeadingController.setPID(3, 0, 0);
        driveWithHeading.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    }

    DoubleSupplier[] arr;

    private SystemState handleStates() {
        return switch (wantedState) {
            case IDLE -> SystemState.IDLE;
            case AUTO -> SystemState.AUTO;
            case TELEOP -> {
                arr = limitVelocity(controller::getLeftY, controller::getLeftX, controller::getRightX, () -> maxSpeed);
                double turnFieldFrame = arr[2].getAsDouble() / maxAngularRate;
                if (Math.abs(turnFieldFrame) > 0.05) {
                    mJoystickLastTouched = Timer.getFPGATimestamp();
                }
                if (Math.abs(turnFieldFrame) > 0.05
                        || (MathUtil.isNear(mJoystickLastTouched, Timer.getFPGATimestamp(), 0.25)
                        && Math.abs(CommandSwerveDrivetrain.getInstance().getState().Speeds.omegaRadiansPerSecond) > Math
                        .toRadians(10)))
                    yield SystemState.TELEOP;
                else {
                    if (mHeadingSetpoint.isEmpty()) {
                        mHeadingSetpoint = Optional.of(CommandSwerveDrivetrain.getInstance().getState().Pose.getRotation());
                    }
                    yield SystemState.KEEP_HEADING;
                }
            }
            case AUTO_PILOT -> SystemState.AUTO_PILOT;
            case AUTO_PILOT_CLIMBING -> SystemState.AUTO_PILOT_CLIMBING;
        };
    }


    private void applyStates() {
        switch (currentSystemState) {
            case IDLE -> idle_state();
            case AUTO -> {
            }
            case TELEOP -> teleop();
            case KEEP_HEADING -> keepHeading();
            case AUTO_PILOT -> autopilot(new Pose2d());
            case AUTO_PILOT_CLIMBING -> autopilot(ScoringManager.getInstance().getClimbingTargetBasedOnCondition());

        }
    }

    private void idle_state() {
        setControl(idleSwerve);
    }

    private void teleop() {
        setControl(driveNoHeading.withVelocityX(arr[0].getAsDouble()).withVelocityY(arr[1].getAsDouble())
                .withRotationalRate(arr[2].getAsDouble()));
        mHeadingSetpoint = Optional.empty();
    }

    private void keepHeading() {
        setControl(driveWithHeading.withVelocityX(arr[0].getAsDouble()).withVelocityY(arr[1].getAsDouble())
                .withTargetDirection(mHeadingSetpoint.get()));
    }

    private void autopilot(Pose2d fieldPosition) {
        setControl(chassisSpeedRequestAutopilot(getChassisSpeedsFromFieldPoint(
                fieldPosition,
                () -> MAX_RELATIVE_AUTOPILOT_VELOCITY_X,
                () -> MAX_RELATIVE_AUTOPILOT_VELOCITY_Y,
                () -> MAX_RELATIVE_AUTOPILOT_VELOCITY_ROT,
                RELATIVE_AUTOPILOT_X_CONTROLLER,
                RELATIVE_AUTOPILOT_Y_CONTROLLER,
                RELATIVE_AUTOPILOT_ROTATIONAL_CONTROLLER)));
        mHeadingSetpoint = Optional.empty();
    }

    private void drive(double xSpeed) {
        setControl(chassisSpeedRequestRobotCentric(new ChassisSpeeds(xSpeed, 0, 0)));
        mHeadingSetpoint = Optional.empty();
    }

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */

    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,        // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null,        // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this
            )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */

    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,        // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null,        // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this
            )
    );

    public double getAvgStatorCurrent() {
        double currrent = 0;
        for (int i = 0; i < 4; i++) {
            currrent += getModules()[i].getDriveMotor().getStatorCurrent().getValueAsDouble();
        }
        return currrent / 4.0;
    }

    public double getAvgSupplyCurrent() {
        double currrent = 0;
        for (int i = 0; i < 4; i++) {
            currrent += getModules()[i].getDriveMotor().getSupplyCurrent().getValueAsDouble();
        }
        return currrent / 4.0;
    }

    public void setChassisSpeeds(ChassisSpeeds speed) {
        setControl(new SwerveRequest.FieldCentric().withRotationalRate(speed.omegaRadiansPerSecond));
    }

    public void pathPlannerResetPose(Pose2d pose) {
//        if (hasUpdatedRotation)
//            resetPose(getState().Pose);
//        else
        resetPose(pose);
    }

    public final SwerveRequest.ApplyRobotSpeeds m_applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();


    void autoCons(ChassisSpeeds speeds, DriveFeedforwards feedforward) {
        if (CommandSwerveDrivetrain.getInstance().wantedState == WantedState.AUTO) {
            CommandSwerveDrivetrain.getInstance().setControl(
                    CommandSwerveDrivetrain.getInstance().m_applyRobotSpeeds.withSpeeds(speeds)
                            .withWheelForceFeedforwardsX(feedforward.robotRelativeForcesXNewtons())
                            .withWheelForceFeedforwardsY(feedforward.robotRelativeForcesYNewtons())
            );
        }
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    () -> getState().Pose,   // Supplier of current robot pose
                    this::pathPlannerResetPose, // Consumer for seeding pose against auto
                    () -> getState().Speeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    this::autoCons,
                    new PPHolonomicDriveController(
                            // PID constants for translation
                            new PIDConstants(12.5, 0, 0.1),//todo 8.5
                            // PID constants for rotation
                            new PIDConstants(11, 0, 0.05)
                    ),
                    config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this// Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */

    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this
            )
    );

    public ChassisSpeeds wantedChassisSpeed() {
        return getKinematics().toChassisSpeeds(getState().ModuleTargets);
    }
    /* The SysId routine to test */

    private final SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    private SwerveRequest chassisSpeedRequestRobotCentric(ChassisSpeeds speeds) {
        return driveNoHeadingRobotCentric.withVelocityX(speeds.vxMetersPerSecond).withVelocityY(speeds.vyMetersPerSecond)
                .withRotationalRate(speeds.omegaRadiansPerSecond);
    }

    private SwerveRequest chassisSpeedRequestAutopilot(ChassisSpeeds speeds) {
        return driveNoHeadingAutopilot.withVelocityX(speeds.vxMetersPerSecond).withVelocityY(speeds.vyMetersPerSecond)
                .withRotationalRate(speeds.omegaRadiansPerSecond);
    }


    private ChassisSpeeds getChassisSpeedsFromFieldPoint(
            Pose2d fieldPoint, DoubleSupplier clampX, DoubleSupplier clampY,
            DoubleSupplier clampRotation, SqrtErrorProfiledPIDController pidX,
            SqrtErrorProfiledPIDController pidY, PIDController pidRot) {
        ChassisSpeeds speeds = new ChassisSpeeds(
                MathUtil.clamp(
                        -pidX.calculate(
                                -getRelativeOffsetFromFieldPoint(fieldPoint).getX()),
                        -clampX.getAsDouble(), clampX.getAsDouble()),

                MathUtil.clamp(
                        -pidY.calculate(
                                -getRelativeOffsetFromFieldPoint(fieldPoint).getY()),
                        -clampY.getAsDouble(), clampY.getAsDouble()),
                MathUtil.clamp(
                        pidRot.calculate(
                                CommandSwerveDrivetrain.getInstance().getState().Pose.getRotation().getRadians(),
                                fieldPoint.getRotation().getRadians()),
                        -clampRotation.getAsDouble(), clampRotation.getAsDouble())
        );
        Translation2d speedsAsTranslation = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        Translation2d correct = speedsAsTranslation.rotateBy(fieldPoint.getRotation());
        return new ChassisSpeeds(correct.getX(), correct.getY(), speeds.omegaRadiansPerSecond);
    }


    public void updatePID(double p, double i, double d) {
        for (int j = 0; j <= 3; j++) {
            getModule(j).getDriveMotor().getConfigurator().apply(new Slot0Configs().withKP(p).withKI(i).withKD(d));
        }
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    private final SwerveRequest.Idle idleSwerve = new SwerveRequest.Idle();

    private final SwerveRequest.RobotCentric driveNoHeadingRobotCentric = new SwerveRequest.RobotCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
    private final SwerveRequest.FieldCentric driveNoHeading = new SwerveRequest.FieldCentric().withDeadband(maxSpeed * 0.06)
            .withRotationalDeadband(maxAngularRate * 0.06)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
    private final SwerveRequest.FieldCentricFacingAngle driveWithHeading = new SwerveRequest.FieldCentricFacingAngle().withDeadband(maxSpeed * 0.06)
            .withRotationalDeadband(maxAngularRate * 0.06)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
    private final SwerveRequest.FieldCentric driveNoHeadingAutopilot = new SwerveRequest.FieldCentric().withDeadband(0.04)
            .withRotationalDeadband(0.03)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
    private static final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private DoubleSupplier[] limitVelocity(DoubleSupplier vxF, DoubleSupplier vyF, DoubleSupplier vrF, DoubleSupplier maxSpeedF) {
        double vx = limitX.calculate(vxF.getAsDouble() * maxSpeedF.getAsDouble() * ALLIANCE_FLIPPER.getAsDouble());
        double vy = limitY.calculate(vyF.getAsDouble() * maxSpeedF.getAsDouble() * ALLIANCE_FLIPPER.getAsDouble());

        double vr = -vrF.getAsDouble() * maxAngularRate;
        double maxSpeed = maxSpeedF.getAsDouble();
        // Convert radius to meters
        double driveRadiusMeters = 14.046417074141907 * 0.0254;

        // Calculate rotational component
        double rotationalComponent = Math.abs(vr * driveRadiusMeters);

        // Calculate current linear magnitude
        double currentLinearMagnitude = Math.sqrt(vx * vx + vy * vy);

        // If no linear velocity, return original values
        if (currentLinearMagnitude == 0) {
            return new DoubleSupplier[]{() -> 0, () -> 0, () -> vr};
        }

        // Calculate total velocity
        double totalVelocity = Math.sqrt(currentLinearMagnitude * currentLinearMagnitude +
                rotationalComponent * rotationalComponent);

        // If we're within speed limit, return original values
        if (totalVelocity <= maxSpeed) {
            return new DoubleSupplier[]{() -> vx, () -> vy, () -> vr};
        }

        // Calculate scaling factor for linear components
        double remainingSpeed = Math.sqrt(maxSpeed * maxSpeed - rotationalComponent * rotationalComponent);
        double scaleFactor = remainingSpeed / currentLinearMagnitude;

        // Scale x and y velocities
        double newVx = vx * scaleFactor;
        double newVy = vy * scaleFactor;

        return new DoubleSupplier[]{() -> newVx, () -> newVy, () -> vr};
    }


    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(drivetrainConstants, OdometryUpdateFrequency, modules);
        configureAutoBuilder();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so user should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     * @param visionStandardDeviation   The standard deviation for vision calculation
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                        setOperatorPerspectiveForward(
                                allianceColor == Alliance.Red ? kRedAlliancePerspectiveRotation
                                        : kBlueAlliancePerspectiveRotation);
                        m_hasAppliedOperatorPerspective = true;
                    }
            );
        }
        log();
        currentSystemState = handleStates();
        applyStates();
    }

    public WantedState getWantedState() {
        return wantedState;
    }

    private void log() {
        Logger.recordOutput("Localization/RobotPose2d", getState().Pose);
        Logger.recordOutput("Swerve/currentSystemState", currentSystemState);
//        Logger.recordOutput("Swerve/States/current\", currentSystemState.toString());\n" +
//                "        Logger.recordOutput(\"Swerve/Modules", getState().ModuleStates);
//        Logger.recordOutput("Swerve/Modules Target", getState().ModuleTargets);


    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    private static CommandSwerveDrivetrain drivetrain;

    public static CommandSwerveDrivetrain getInstance() {
        return drivetrain;
    }

    public static CommandSwerveDrivetrain init(CommandSwerveDrivetrain drivetrain) {
        CommandSwerveDrivetrain.drivetrain = drivetrain;
        return drivetrain;
    }
}
