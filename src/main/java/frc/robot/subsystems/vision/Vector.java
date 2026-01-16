package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Translation2d;

public class Vector {

    private Translation2d pointA;
    private Translation2d pointB;

    public Vector(Translation2d pointA, Translation2d pointB) {
        this.pointA = pointA;
        this.pointB = pointB;
    }

    public Translation2d[] getFunction(){
        Translation2d funcVector = new Translation2d(pointB.getX()-pointA.getX(), pointB.getY()-pointA.getY());
        Translation2d[] arr = {pointA, funcVector};
        return arr;
    }

    public Translation2d findIntersectionPoints(Vector vector){
        double x = getFunction()[0].getX()-vector.getFunction()[0].getX();
        double[] tVector = {x/vector.getFunction()[1].getX(), getFunction()[1].getX()/vector.getFunction()[1].getX()};
        double t = (getFunction()[0].getY()-vector.getFunction()[1].getY()*tVector[0])/(vector.getFunction()[1].getY()*tVector[1]-getFunction()[1].getY());
        return new Translation2d(getFunction()[0].getX()+t*getFunction()[1].getX(), getFunction()[0].getY()+t*getFunction()[1].getY());
    }

}
