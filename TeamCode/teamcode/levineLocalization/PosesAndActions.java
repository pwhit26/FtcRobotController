package org.firstinspires.ftc.teamcode.levineLocalization;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class PosesAndActions {
    public Pose2d pose;
    public String action;
    public PosesAndActions(Pose2d pose, String action){
        this.pose = pose;
        this.action = action;
    }
    public static boolean isRealAction(String action){
        return !action.equals("");
    }
    public String toString(){
        return "Pose is " + pose + " action is " + action;
    }
}
