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
        return 
    }

}
