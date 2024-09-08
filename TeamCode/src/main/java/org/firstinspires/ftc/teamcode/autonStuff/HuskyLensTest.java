
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
    public static double xPosBlue = 0, yPosBlue = -20, headingBlue = 90;
    public static double xPosWhite = -20, yPosWhite = 0, headingWhite = 90;
    public static double xPosRed = 20, yPosRed = 0, headingRed = 90;
    public static double maxVel = 40;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        ArrayList<PosesAndActions> pointToGoTo = new ArrayList<>();
        Pose2d startPos = new Pose2d(xPosStart, yPosStart, Math.toRadians(headingStart));
        Pose2d bluePos = new Pose2d(xPosBlue, yPosBlue, Math.toRadians(headingBlue));
        Pose2d whitePos = new Pose2d(xPosWhite, yPosWhite, Math.toRadians(headingWhite));
        Pose2d redPos = new Pose2d(xPosRed, yPosRed, Math.toRadians(headingRed));
        bot.init();
        int colorID = 0;
        pointToGoTo.add(new PosesAndActions(startPos, ""));
        while (opModeInInit()) {
            int largestArea = 0;
            int position = 0;
            int newArea;
            HuskyLens.Block[] blocks = bot.huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
            }
            for (int k = 0; k < blocks.length; k++) {
                newArea = blocks[k].height * blocks[k].width;
                if (newArea > largestArea) {
                    largestArea = newArea;
                    position = k;
                }
            }

            if (blocks[position].id == 1) {
                telemetry.addLine("blue");
                colorID = 1;
            }
            if (blocks[position].id == 2) {
                telemetry.addLine("white");
               colorID = 2;
            }
            if (blocks[position].id == 3) {
                telemetry.addLine("red");
                colorID = 3;
            }
        }
        if (colorID == 1)
        {
            pointToGoTo.add(new PosesAndActions(bluePos, ""));
        }
        else if (colorID == 2)
        {
            pointToGoTo.add(new PosesAndActions(whitePos, ""));
        }
        else if (colorID == 3)
        {
            pointToGoTo.add(new PosesAndActions(redPos, ""));
        }
        pointToGoTo.add(new PosesAndActions(startPos, ""));
        follower.init(pointToGoTo, false, true);
        follower.goToPoints(true, maxVel);
    }
}
            //blue = 1 white = 2 red = 3

