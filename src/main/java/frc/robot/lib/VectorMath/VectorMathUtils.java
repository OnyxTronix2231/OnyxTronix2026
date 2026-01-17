package frc.robot.lib.VectorMath;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.Point;

public class VectorMathUtils {

    public static LineFunction getFunction(frc.robot.lib.Point pointA, frc.robot.lib.Point pointB) {
        return new LineFunction(new Translation2d(pointB.x()-pointA.x(),pointB.y()-pointA.y()), new Translation2d(pointA.x(), pointA.y()));
    }

    public static frc.robot.lib.Point findPoint(LineFunction lineFunction, double t) {
        return new frc.robot.lib.Point(lineFunction.getNumCoefficient().getX() + lineFunction.getTCoefficient().getX() * t, lineFunction.getNumCoefficient().getY() + lineFunction.getTCoefficient().getY() * t);
    }

    public static Point findIntersectionPoint(LineFunction lineFunction1, LineFunction lineFunction2) {
        double x = lineFunction2.getNumCoefficient().getX() - lineFunction1.getNumCoefficient().getX();
        LineFunction tFunction = new LineFunction(lineFunction2.getTCoefficient().getX() / lineFunction1.getTCoefficient().getX(), x / lineFunction1.getTCoefficient().getX());
        double y = lineFunction2.getNumCoefficient().getY() - lineFunction1.getNumCoefficient().getY() - (tFunction.getOneCoefficient() * lineFunction1.getTCoefficient().getY());
        double sCoefficient = lineFunction1.getTCoefficient().getY() * tFunction.getXCoefficient() - lineFunction2.TCoefficient.getY();
        double s = y / sCoefficient;
        return findPoint(lineFunction2, s);
    }
}
