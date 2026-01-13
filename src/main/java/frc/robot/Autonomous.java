package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class Autonomous {

    private final SendableChooser<Command> autoChooser;

    public Autonomous() {
        autoChooser = AutoBuilder.buildAutoChooser();
        addAllEvents();
        addAllTheAuto();
        autoChooser.setDefaultOption("Example Auto", new PathPlannerAuto("Example Auto"));
        Shuffleboard.getTab("auto").add(autoChooser);
    }

    private void addAuto(String name) {
        autoChooser.addOption(name,
                new PathPlannerAuto(name)
        );
    }

    public void addAllEvents() {
//        NamedCommands.registerCommand("name", new InstantCommand(() -> the command you want));
    }

    public void addAllTheAuto() {
        addAuto("Example Auto");
    }

    public Command getSelectedCommand() {
        return autoChooser.getSelected();
    }
}
