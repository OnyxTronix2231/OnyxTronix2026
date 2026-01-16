package frc.robot.visualization;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.hood.Hood;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import static frc.robot.subsystems.hood.HoodConstants.HOOD_SUBSYSTEM_NAME;

public class HoodVisualization extends VisualizedSubsystem {
    private final Hood hood;

    public HoodVisualization() {
        this.hood = Hood.getInstance();
    }

    @Override
    void updateVisualization() {
        HoodVisualizationMechanism.HOOD.setAngle(hood.getAngle());
    }

    public class HoodVisualizationMechanism {
        private static final double HOOD_X_POSITION = 2;
        private static final double HOOD_Y_POSITION = 1;
        public static final LoggedMechanismRoot2d hoodRoot = ROBOT_MECHANISM.getRoot(HOOD_SUBSYSTEM_NAME, HOOD_X_POSITION, HOOD_Y_POSITION);

        private static final double HOOD_LIGAMENT_LENGTH = 1;
        private static final double HOOD_LIGAMENT_ANGLE = 0;
        private static final double HOOD_LIGAMENT_LINE_WIDTH = 8;
        public static final Color8Bit HOOD_LIGAMENT_COLOR = new Color8Bit(Color.kOrange);

        public static final LoggedMechanismLigament2d HOOD = hoodRoot.append(
            new LoggedMechanismLigament2d(
                    HOOD_SUBSYSTEM_NAME,
                    HOOD_LIGAMENT_LENGTH,
                    HOOD_LIGAMENT_ANGLE,
                    HOOD_LIGAMENT_LINE_WIDTH,
                    HOOD_LIGAMENT_COLOR
            ));

    }
}
