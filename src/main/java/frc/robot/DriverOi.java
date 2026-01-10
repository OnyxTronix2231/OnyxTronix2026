package frc.robot;

import commandControl.CommandConsoleController;
import commandControl.CommandPlaystation5Controller;

public class DriverOi {
    public static CommandConsoleController controller = new CommandPlaystation5Controller(0);

}
