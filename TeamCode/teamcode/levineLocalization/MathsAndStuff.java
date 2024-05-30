package org.firstinspires.ftc.teamcode.levineLocalization;

public class MathsAndStuff {
    public static double AngleWrap(double angle){
        while(angle < -Math.PI){
            angle += 2 * Math.PI;
        }
        while(angle > Math.PI){
            angle -= 2 * Math.PI;
        }
        return angle;
    }
    public static double AngleWrapRoadrunner(double angle){
        while(angle < 0){
            angle += 2 * Math.PI;
        }
        while(angle > 2*Math.PI){
            angle -= 2 * Math.PI;
        }
        return angle;
    }
    public static double AngleWrapDeg(double angle){
        while(angle < 0){
            angle += 360;
        }
        while(angle > 360){
            angle -= 360;
        }
        return angle;
    }
}
