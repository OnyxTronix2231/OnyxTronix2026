package frc.robot.lib.PID;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class PIDValues {
    private double kP;
    private double kI;
    private double kD;
    private double kG;
    private double kS;
    private double kV;
    private double kA;
    private GravityTypeValue gravityType;
    private StaticFeedforwardSignValue staticFeedforwardSignValue;

    public PIDValues(double kP, double kI, double kD, double kG, double kS, double kV, double kA,
                     GravityTypeValue gravityType, StaticFeedforwardSignValue staticFeedforwardSignValue) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kG = kG;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.gravityType = gravityType;
        this.staticFeedforwardSignValue = staticFeedforwardSignValue;
    }

    public double getkP() {
        return kP;
    }

    public void setkP(double kP) {
        this.kP = kP;
    }

    public double getkI() {
        return kI;
    }

    public void setkI(double kI) {
        this.kI = kI;
    }

    public double getkD() {
        return kD;
    }

    public void setkD(double kD) {
        this.kD = kD;
    }

    public double getkG() {
        return kG;
    }

    public void setkG(double kG) {
        this.kG = kG;
    }

    public double getkS() {
        return kS;
    }

    public void setkS(double kS) {
        this.kS = kS;
    }

    public double getkV() {
        return kV;
    }

    public void setkV(double kV) {
        this.kV = kV;
    }

    public double getkA() {
        return kA;
    }

    public void setkA(double kA) {
        this.kA = kA;
    }

    public GravityTypeValue getGravityType() {
        return gravityType;
    }

    public void setGravityType(GravityTypeValue gravityType) {
        this.gravityType = gravityType;
    }

    public StaticFeedforwardSignValue getStaticFeedforwardSignValue() {
        return staticFeedforwardSignValue;
    }

    public void setStaticFeedforwardSignValue(StaticFeedforwardSignValue staticFeedforwardSignValue) {
        this.staticFeedforwardSignValue = staticFeedforwardSignValue;
    }

    public void updatePIDValues(TalonFX motor) {
        motor.getConfigurator().apply(new Slot0Configs()
                .withKP(kP)
                .withKI(kI)
                .withKD(kD)
                .withKG(kG)
                .withKS(kS)
                .withKV(kV)
                .withKA(kA)
                .withGravityType(gravityType)
                .withStaticFeedforwardSign(staticFeedforwardSignValue));
    }

    public Slot0Configs pidValuesToSlot0Configs() {
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.withKP(kP);
        slot0Configs.withKI(kI);
        slot0Configs.withKD(kD);
        slot0Configs.withKG(kG);
        slot0Configs.withKS(kS);
        slot0Configs.withKV(kV);
        slot0Configs.withKA(kA);
        slot0Configs.withGravityType(gravityType);
        slot0Configs.withStaticFeedforwardSign(staticFeedforwardSignValue);
        return slot0Configs;
    }

    public Slot1Configs pidValuesToSlot1Configs() {
        Slot1Configs slot1Configs = new Slot1Configs();
        slot1Configs.withKP(kP);
        slot1Configs.withKI(kI);
        slot1Configs.withKD(kD);
        slot1Configs.withKG(kG);
        slot1Configs.withKS(kS);
        slot1Configs.withKV(kV);
        slot1Configs.withKA(kA);
        slot1Configs.withGravityType(gravityType);
        slot1Configs.withStaticFeedforwardSign(staticFeedforwardSignValue);
        return slot1Configs;
    }
}
