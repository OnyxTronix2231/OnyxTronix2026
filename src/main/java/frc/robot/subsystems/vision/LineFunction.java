package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Translation2d;

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

    public double getxCoefficient() {
        return xCoefficient;
    }

    public double getOneCoefficient() {
        return oneCoefficient;
    }

    public LineFunction(double xCoefficient, double oneCoefficient) {
        this.xCoefficient = xCoefficient;
        this.oneCoefficient = oneCoefficient;
    }

    public Translation2d getNumCoefficient() {
        return vectorCoefficient;
    }

    public Translation2d findIntersectionPoint(LineFunction lineFunction){
        double x = lineFunction.getNumCoefficient().getX()-getNumCoefficient().getX();
        LineFunction tFunction = new LineFunction(lineFunction.getTCoefficient().getX()/getTCoefficient().getX(), x/getTCoefficient().getX());
    }
}
