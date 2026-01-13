package frc.robot.subsystems.arc;

import frc.robot.lib.OnyxMotorInputs;
import frc.robot.lib.PID.PIDValues;

public interface ArcIO {
    void updateInputs(ArcInputs inputs);

    class ArcInputs {
        public OnyxMotorInputs arcMotorInputs;
    }

    void setDutyCycle(double dutyCycle);

    void updatePID(PIDValues pidValues);

    void moveToAngle(double angle);
}
