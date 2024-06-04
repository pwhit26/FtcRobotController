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
public class ParkerAuton extends LinearOpMode {
    public HaolinIsStupid bot = new HaolinIsStupid(this);
    ActionRunnerCenterStageAuton SpencerLevinePoopyHead = new ActionRunnerCenterStageAuton(this, bot);
    PointFollower follower = new PointFollower(this, SpencerLevinePoopyHead);
    public static double xPosStart = 0, yPosStart = 0, headingStart = 90;
    public static double xPosEnd = -10, yPosEnd = 20, headingEnd = 90;
    public static double maxVel = 20;
    @Override
    public void runOpMode() throws InterruptedException {
        ArrayList<PosesAndActions> pointToGoTo = new ArrayList<>();
        Pose2d startPos = new Pose2d(xPosStart, yPosStart, Math.toRadians(headingStart));
        Pose2d endPos = new Pose2d(xPosEnd, yPosEnd, Math.toRadians(headingEnd));
        bot.init();
        pointToGoTo.add(new PosesAndActions(startPos, ""));
        pointToGoTo.add(new PosesAndActions(endPos, ""));
        follower.init(pointToGoTo, false, true);
        waitForStart();
        follower.goToPoints(true, maxVel);
        pointToGoTo.clear();
        pointToGoTo.add(new PosesAndActions(startPos, ""));
        follower.reinit(pointToGoTo);
        follower.goToPoints(true, maxVel);
    }
}
