package frc.robot.subsystems.swerve;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SwerveShuffleBoard {
    public SwerveShuffleBoard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Autopilot");
        GenericEntry pYcontroller = tab.add("pYcontroller", 0).getEntry();
        GenericEntry iYcontroller = tab.add("iYcontroller", 0).getEntry();
        GenericEntry dYcontroller = tab.add("dYcontroller", 0).getEntry();
        tab.add("update_RELATIVE_AUTOPILOT_Y_CONTROLLER_PID", new InstantCommand(() -> AutopilotConstants.update_RELATIVE_AUTOPILOT_Y_CONTROLLER_PID(pYcontroller.getDouble(0), iYcontroller.getDouble(0), dYcontroller.getDouble(0))));
        GenericEntry pXcontroller = tab.add("pXcontroller", 0).getEntry();
        GenericEntry iXcontroller = tab.add("iXcontroller", 0).getEntry();
        GenericEntry dXcontroller = tab.add("dXcontroller", 0).getEntry();
        tab.add("update_RELATIVE_AUTOPILOT_X_CONTROLLER_PID", new InstantCommand(() -> AutopilotConstants.update_RELATIVE_AUTOPILOT_X_CONTROLLER_PID(pXcontroller.getDouble(0), iXcontroller.getDouble(0), dXcontroller.getDouble(0))));
        GenericEntry pRcontroller = tab.add("pRcontroller", 0).getEntry();
        GenericEntry iRcontroller = tab.add("iRcontroller", 0).getEntry();
        GenericEntry dRcontroller = tab.add("dRcontroller", 0).getEntry();
        tab.add("update_RELATIVE_AUTOPILOT_ROTATIONAL_CONTROLLER_PID", new InstantCommand(() -> AutopilotConstants.update_RELATIVE_AUTOPILOT_ROTATIONAL_CONTROLLER_PID(pRcontroller.getDouble(0), iRcontroller.getDouble(0), dRcontroller.getDouble(0))));
        GenericEntry YmaxSpeed = tab.add("YmaxSpeed", 0).getEntry();
        tab.add("setMAX_RELATIVE_AUTOPILOT_VELOCITY_Y", new InstantCommand(() -> AutopilotConstants.setMAX_RELATIVE_AUTOPILOT_VELOCITY_Y(YmaxSpeed.getDouble(0))));
        GenericEntry XmaxSpeed = tab.add("XmaxSpeed", 0).getEntry();
        tab.add("setMAX_RELATIVE_AUTOPILOT_VELOCITY_X", new InstantCommand(() -> AutopilotConstants.setMAX_RELATIVE_AUTOPILOT_VELOCITY_X(XmaxSpeed.getDouble(0))));
        GenericEntry RmaxSpeed = tab.add("RmaxSpeed", 0).getEntry();
        tab.add("setMAX_RELATIVE_AUTOPILOT_VELOCITY_ROT", new InstantCommand(() -> AutopilotConstants.setMAX_RELATIVE_AUTOPILOT_VELOCITY_ROT(RmaxSpeed.getDouble(0))));
        tab.addDouble("yErr", () -> AutopilotConstants.yErr());
        tab.addDouble("xErr", () -> AutopilotConstants.xErr());
        tab.addDouble("rotErr", () -> AutopilotConstants.rotErr());

    }
}
