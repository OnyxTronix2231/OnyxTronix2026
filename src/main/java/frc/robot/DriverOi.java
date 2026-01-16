package frc.robot;

import commandControl.CommandConsoleController;
import commandControl.CommandPlaystation5Controller;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.data.ScoringData;
import frc.robot.data.ScoringManager;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

import java.util.Optional;

import static frc.robot.data.ScoringManager.climbingCondition;

public class DriverOi {
    public static CommandConsoleController controller = new CommandPlaystation5Controller(0);
    public static CommandConsoleController controllerDeputy = new CommandPlaystation5Controller(1);

    private static void resetSwerveAngle() {
        CommandSwerveDrivetrain.getInstance().tareEverything();
        CommandSwerveDrivetrain.getInstance().mHeadingSetpoint = Optional.of(CommandSwerveDrivetrain.getInstance().getState().Pose.getRotation());
    }

    public DriverOi withSwerve() {
        controller.centerLeft().onTrue(new InstantCommand(DriverOi::resetSwerveAngle));
        return this;
    }

    public DriverOi withOneDriver() {
        controller.centerLeft().onTrue(new InstantCommand(DriverOi::resetSwerveAngle));

        controller.povLeft().onTrue(new InstantCommand(() -> ScoringManager.getInstance().setClimbingSide(ScoringData.ClimbingSide.LEFT)));
        controller.povRight().onTrue(new InstantCommand(() -> ScoringManager.getInstance().setClimbingSide(ScoringData.ClimbingSide.RIGHT)));
        controller.leftTrigger().onTrue(new InstantCommand(() -> CommandSwerveDrivetrain.getInstance().setWantedState(CommandSwerveDrivetrain.WantedState.AUTO_PILOT_CLIMBING)));
        controller.bumperLeft().onTrue(new InstantCommand(() -> CommandSwerveDrivetrain.getInstance().setWantedState(CommandSwerveDrivetrain.WantedState.TELEOP)));
        controller.buttonUp().onTrue(new InstantCommand(() -> climbingCondition = !climbingCondition));
        return this;
    }

}
