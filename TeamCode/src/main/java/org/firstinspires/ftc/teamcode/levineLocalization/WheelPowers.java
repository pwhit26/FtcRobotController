package org.firstinspires.ftc.teamcode.levineLocalization;

public class WheelPowers {
    public double frp;
    public double flp;
    public double brp;
    public double blp;

    public WheelPowers(double fr, double fl, double br, double bl){
        frp = fr;
        flp = fl;
        brp = br;
        blp = bl;
    }
    public String toString() {
        return "Wheel Powers: fr: " + frp + " fl: " + flp + " br: " + brp + " bl: " + blp;
    }
}
