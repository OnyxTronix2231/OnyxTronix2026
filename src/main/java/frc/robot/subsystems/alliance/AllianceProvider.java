package frc.robot.subsystems.alliance;

import edu.wpi.first.wpilibj.DriverStation;

public class AllianceProvider {

    private static AllianceProvider instance;

    public static AllianceProvider getInstance() {
        if (instance == null)
            instance = new AllianceProvider();
        return instance;
    }

    public boolean isBlueAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Red).equals(DriverStation.Alliance.Blue);
    }
}
