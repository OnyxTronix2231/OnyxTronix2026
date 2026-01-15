package frc.robot.visualization;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.arc.Arc;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import static frc.robot.subsystems.arc.ArcConstants.ARC_SUBSYSTEM_NAME;

public class ArcVisualization extends VisualizedSubsystem {
    private final Arc arc;

    public ArcVisualization() {
        this.arc = Arc.getInstance();
    }

    @Override
    void updateVisualization() {
        ArcVisualizationMechanism.ARC.setAngle(arc.getAngle());
    }

    public class ArcVisualizationMechanism {
        private static final double ARC_X_POSITION = 2;
        private static final double ARC_Y_POSITION = 1;
        public static final LoggedMechanismRoot2d arcRoot = ROBOT_MECHANISM.getRoot(ARC_SUBSYSTEM_NAME, ARC_X_POSITION, ARC_Y_POSITION);

        private static final double ARC_LIGAMENT_LENGTH = 1;
        private static final double ARC_LIGAMENT_ANGLE = 0;
        private static final double ARC_LIGAMENT_LINE_WIDTH = 8;
        public static final Color8Bit ARC_LIGAMENT_COLOR = new Color8Bit(Color.kOrange);

        public static final LoggedMechanismLigament2d ARC = arcRoot.append(
            new LoggedMechanismLigament2d(
                "arc",
                ARC_LIGAMENT_LENGTH,
                ARC_LIGAMENT_ANGLE,
                ARC_LIGAMENT_LINE_WIDTH,
                ARC_LIGAMENT_COLOR
            ));

    }
}
