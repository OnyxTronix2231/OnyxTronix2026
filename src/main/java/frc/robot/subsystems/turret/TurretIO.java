package frc.robot.subsystems.turret;

import frc.robot.lib.OnyxMotorInputs;
import frc.robot.lib.PID.PIDValues;

public interface TurretIO {

    void updateInputs(TurretInputs inputs);

    class TurretInputs {
        public double encoderPosition;

        public OnyxMotorInputs turretMotorInputs;
    }

    void setDutyCycle(double dutyCycle);

    void moveToAngle(double angle);

    void updatePID(PIDValues pidValues);
}
