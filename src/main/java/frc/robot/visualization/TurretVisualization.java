package frc.robot.visualization;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.turret.Turret;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class TurretVisualization extends VisualizedSubsystem {

    private final Turret turret;

    public TurretVisualization() {
        turret = Turret.getInstance();
    }

    @Override
    void updateVisualization() {
        TurretVisualizationMechanism.TURRET.setAngle(turret.getTurretAngle());
    }

    public class TurretVisualizationMechanism {
        private static final double TURRET_X_POSITION = 3;
        private static final double TURRET_Y_POSITION = 0.1;
        public static final LoggedMechanismRoot2d turretRoot = ROBOT_MECHANISM.getRoot("turret", TURRET_X_POSITION, TURRET_Y_POSITION);

        private static final double TURRET_LIGAMENT_LENGTH = 0.3;
        private static final double TURRET_LIGAMENT_ANGLE = 180;
        private static final double TURRET_LIGAMENT_LINE_WIDTH = 10;
        public static final Color8Bit TURRET_LIGAMENT_COLOR = new Color8Bit(Color.kRed);
        public static final LoggedMechanismLigament2d TURRET = turretRoot.append(
                new LoggedMechanismLigament2d(
                        "TURRET",
                        TURRET_LIGAMENT_LENGTH, TURRET_LIGAMENT_ANGLE,
                        TURRET_LIGAMENT_LINE_WIDTH, TURRET_LIGAMENT_COLOR
                )
        );
    }
}
