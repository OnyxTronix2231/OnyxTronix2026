package frc.robot.data;

import edu.wpi.first.wpilibj.DriverStation;

public class HubStatusData {
    private boolean active;
    private double timeLeft;

    public HubStatusData() {
        active = false;
        timeLeft = 0;
    }

    public boolean isActive() {
        return active;
    }

    public void setActive(boolean active) {
        this.active = active;
    }

    public double getTimeLeft() {
        return timeLeft;
    }

    public void setTimeLeft(double timeLeft) {
        this.timeLeft = timeLeft;
    }

    public void updateHubStatusData() {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        String gameData = DriverStation.getGameSpecificMessage();
        if (!gameData.isEmpty()) {
            char inactiveFirst = gameData.charAt(0);
        }
    }
}
