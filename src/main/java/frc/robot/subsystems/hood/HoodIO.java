package frc.robot.subsystems.hood;

import frc.robot.lib.OnyxMotorInputs;
import frc.robot.lib.PID.PIDValues;

public interface HoodIO {
    void updateInputs(HoodInputs inputs);

    class HoodInputs {
        public OnyxMotorInputs hoodMotorInputs;

        public double cancoderPosition;
    }

    void setDutyCycle(double dutyCycle);

    void updatePID(PIDValues pidValues);

    void moveToAngle(double angle);
}
