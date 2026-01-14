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
        autoChooser.setDefaultOption("prior 1", new PathPlannerAuto("prior 1"));
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
        addAuto("prior 1");
        addAuto("prior 2");
        addAuto("prior 3");
        addAuto("prior 4");
        addAuto("prior 5");
    }

    public Command getSelectedCommand() {
        return autoChooser.getSelected();
    }
}
