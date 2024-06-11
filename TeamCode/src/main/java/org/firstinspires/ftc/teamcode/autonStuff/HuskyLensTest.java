
package org.firstinspires.ftc.teamcode.autonStuff;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HaolinIsStupid;
import org.firstinspires.ftc.teamcode.levineLocalization.ActionRunnerCenterStageAuton;
import org.firstinspires.ftc.teamcode.levineLocalization.PointFollower;
import org.firstinspires.ftc.teamcode.levineLocalization.PosesAndActions;

import java.util.ArrayList;

@Config
@TeleOp
public class HuskyLensTest extends LinearOpMode {
    public HaolinIsStupid bot = new HaolinIsStupid(this);
    ActionRunnerCenterStageAuton SpencerLevinePoopyHead = new ActionRunnerCenterStageAuton(this, bot);
    PointFollower follower = new PointFollower(this, SpencerLevinePoopyHead);
    public static double xPosStart = 0, yPosStart = 0, headingStart = 90;
    public static double xPosBlue1 = 0, yPosBlue1 = 0, headingBlue1 = 180;
    public static double xPosBlue2 = 0, yPosBlue2 = 0, headingBlue2 = 270;
    public static double xPosBlue3 = 0, yPosBlue3 = 0, headingBlue3 = 360;
    public static double xPosWhite = 0, yPosWhite = 20, headingWhite = 90;
    public static double xPosRed = 0, yPosRed = -20, headingRed = 90;
    public static double maxVel = 20;

    @Override
    public void runOpMode() throws InterruptedException {
        ArrayList<PosesAndActions> pointToGoTo = new ArrayList<>();
        Pose2d startPos = new Pose2d(xPosStart, yPosStart, Math.toRadians(headingStart));
        Pose2d blue1Pos = new Pose2d(xPosBlue1, yPosBlue1, Math.toRadians(headingBlue1));
        Pose2d blue2Pos = new Pose2d(xPosBlue2, yPosBlue2, Math.toRadians(headingBlue2));
        Pose2d blue3Pos = new Pose2d(xPosBlue3, yPosBlue3, Math.toRadians(headingBlue3));
        Pose2d whitePos = new Pose2d(xPosWhite, yPosWhite, Math.toRadians(headingWhite));
        Pose2d redPos = new Pose2d(xPosRed, yPosRed, Math.toRadians(headingRed));
        bot.init();
        waitForStart();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        while (opModeIsActive()) {
            int largestArea = 0;
            int position = 0;
            HuskyLens.Block[] blocks = bot.huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
            }
            for (int k = 0; k < blocks.length; k++) {
                int newArea = blocks[k].height * blocks[k].width;
                if (newArea > largestArea) {
                    largestArea = newArea;
                    position = k;
                }
            }
            if (blocks[position].id == 1) {
                telemetry.addLine("blue");
                pointToGoTo.add(new PosesAndActions(startPos, ""));
                pointToGoTo.add(new PosesAndActions(blue1Pos, ""));
                pointToGoTo.add(new PosesAndActions(blue2Pos, ""));
                pointToGoTo.add(new PosesAndActions(blue3Pos, ""));
                follower.init(pointToGoTo, false, true);
                waitForStart();
                follower.goToPoints(true, maxVel);
                pointToGoTo.clear();
                pointToGoTo.add(new PosesAndActions(startPos, ""));
                follower.reinit(pointToGoTo);
                follower.goToPoints(true, maxVel);
                break;
            }
            if (blocks[position].id == 2) {
                telemetry.addLine("white");
                pointToGoTo.add(new PosesAndActions(startPos, ""));
                pointToGoTo.add(new PosesAndActions(whitePos, ""));
                follower.init(pointToGoTo, false, true);
                waitForStart();
                follower.goToPoints(true, maxVel);
                pointToGoTo.clear();
                pointToGoTo.add(new PosesAndActions(startPos, ""));
                follower.reinit(pointToGoTo);
                follower.goToPoints(true, maxVel);
                break;
            }
            if (blocks[position].id == 3) {
                telemetry.addLine("red");
                pointToGoTo.add(new PosesAndActions(startPos, ""));
                pointToGoTo.add(new PosesAndActions(redPos, ""));
                follower.init(pointToGoTo, false, true);
                waitForStart();
                follower.goToPoints(true, maxVel);
                pointToGoTo.clear();
                pointToGoTo.add(new PosesAndActions(startPos, ""));
                follower.reinit(pointToGoTo);
                follower.goToPoints(true, maxVel);
                break;
            }
        }
    }
}
            //blue = 1 white = 2 red = 3

