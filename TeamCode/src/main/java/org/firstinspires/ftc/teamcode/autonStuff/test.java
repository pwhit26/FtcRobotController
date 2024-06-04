package org.firstinspires.ftc.teamcode.autonStuff;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HaolinIsStupid;
import org.firstinspires.ftc.teamcode.levineLocalization.ActionRunnerCenterStageAuton;
import org.firstinspires.ftc.teamcode.levineLocalization.PointFollower;
import org.firstinspires.ftc.teamcode.levineLocalization.PosesAndActions;

import java.util.ArrayList;

@Autonomous
@Config
public class test extends LinearOpMode {
    public HaolinIsStupid bot = new HaolinIsStupid(this);
    ActionRunnerCenterStageAuton SpencerLevinePoopyHead = new ActionRunnerCenterStageAuton(this,bot);
    PointFollower follower=new PointFollower(this, SpencerLevinePoopyHead);
    public static double maxVel=20;
    public static double xPosStart=60, yPosStart=35, headingStart=3.14;
    public static double xPos1=27.8, yPos1=34.4, heading1=3.14;
    public static double xPos2=5.5, yPos2=20.9, heading2=3.62;
    public static double xPos3=5.3, yPos3=17.7, heading3=4.72;
    public static double xPos4=55, yPos4=11.6, heading4=4.71;
    public static double xPosEnd=55, yPosEnd=35, headingEnd=3.14;



    @Override
    public void runOpMode() throws InterruptedException {
        ArrayList<PosesAndActions> pointToGoTo = new ArrayList<>();
        bot.init();
        Pose2d startPos= new Pose2d(xPosStart, yPosStart, headingStart);
        Pose2d Pos1= new Pose2d(xPos1, yPos1, heading1);
        Pose2d Pos2= new Pose2d(xPos2, yPos2, heading2);
        Pose2d Pos3= new Pose2d(xPos3, yPos3, heading3);
        Pose2d Pos4= new Pose2d(xPos4, yPos4, heading4);
        Pose2d endPos= new Pose2d(xPosEnd, yPosEnd, headingEnd);
        pointToGoTo.add(new PosesAndActions(startPos, ""));
        pointToGoTo.add(new PosesAndActions(Pos1, ""));
        follower.init(pointToGoTo, false, true);
        waitForStart();
        follower.goToPoints(true, maxVel);
        pointToGoTo.clear();
        pointToGoTo.add(new PosesAndActions(Pos2, ""));
        pointToGoTo.add(new PosesAndActions(Pos3, ""));
        pointToGoTo.add(new PosesAndActions(Pos4, ""));
        pointToGoTo.add(new PosesAndActions(endPos, ""));
        follower.reinit(pointToGoTo);
        follower.goToPoints(true, maxVel);
    }
}
