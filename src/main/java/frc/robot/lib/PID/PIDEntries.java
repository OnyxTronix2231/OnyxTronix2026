package frc.robot.lib.PID;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class PIDEntries {

    private GenericEntry kP;
    private GenericEntry kI;
    private GenericEntry kD;
    private GenericEntry kG;
    private GenericEntry kS;
    private GenericEntry kV;
    private GenericEntry kA;
    private final PIDValues defaultValues;

    public PIDEntries(String tabName, String prefix, PIDValues defaultValues) {
        this.defaultValues = defaultValues;
        setPIDEntries(tabName, prefix);
    }


    private void setPIDEntries(String tabName, String prefix) {
        ShuffleboardTab tab = Shuffleboard.getTab(tabName);
        this.kP = tab.add(prefix + " kP", defaultValues.getkP()).getEntry();
        this.kI = tab.add(prefix + " kI", defaultValues.getkI()).getEntry();
        this.kD = tab.add(prefix + " kD", defaultValues.getkD()).getEntry();
        this.kG = tab.add(prefix + " kG", defaultValues.getkG()).getEntry();
        this.kS = tab.add(prefix + " kS", defaultValues.getkS()).getEntry();
        this.kV = tab.add(prefix + " kV", defaultValues.getkV()).getEntry();
        this.kA = tab.add(prefix + " kA", defaultValues.getkA()).getEntry();
    }

    public PIDValues getPIDValues() {
        return new PIDValues(
                kP.getDouble(defaultValues.getkP()),
                kI.getDouble(defaultValues.getkI()),
                kD.getDouble(defaultValues.getkD()),
                kG.getDouble(defaultValues.getkG()),
                kS.getDouble(defaultValues.getkS()),
                kV.getDouble(defaultValues.getkV()),
                kA.getDouble(defaultValues.getkA()),
                defaultValues.getGravityType(),
                defaultValues.getStaticFeedforwardSignValue()
        );
    }
}

