package frc.robot.lib.VectorMath;

import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.Logger;

public class LineFunction {
    public Translation2d TCoefficient;
    public Translation2d vectorCoefficient;
    public double xCoefficient;
    public double oneCoefficient;

    public LineFunction(Translation2d TCoefficient, Translation2d vectorCoefficient) {
        this.TCoefficient = TCoefficient;
        this.vectorCoefficient = vectorCoefficient;
    }

    public Translation2d getTCoefficient() {
        return TCoefficient;
    }

    public double getXCoefficient() {
        return xCoefficient;
    }

    public double getOneCoefficient() {
        return oneCoefficient;
    }

    public LineFunction(double xCoefficient, double oneCoefficient) {
        this.xCoefficient = xCoefficient;
        this.oneCoefficient = oneCoefficient;
    }

    public static void log(){
        Logger.recordOutput("VectorTest", VectorMathUtils.findIntersectionPoint(new LineFunction(new Translation2d(2,-1), new Translation2d(0,3)), new LineFunction(new Translation2d(-1,2), new Translation2d(4,-1))));
    }

    public Translation2d getNumCoefficient() {
        return vectorCoefficient;
    }

}
