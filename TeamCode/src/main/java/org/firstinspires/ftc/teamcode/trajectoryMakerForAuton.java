package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.*;

import java.util.*;

/**
 * This is a simple teleop routine for making big men. Drive the robot around like a normal
 * auton routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to make an auton trajectory(note: David is not smart enough to do this).
 */
@Config
@TeleOp(group = "drive")
//@Disabled
public class trajectoryMakerForAuton extends LinearOpMode {

    //35, 63
    //-36, 63
    //
    public static double startingPosx = 14;

    public static double startingPosy = 58;

    public static double startingHeading = -Math.toRadians(90);
    HaolinIsStupid wBot = new HaolinIsStupid(this);


    @Override
    public void runOpMode() throws InterruptedException {
        wBot.init();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Pose2d startingPosition = new Pose2d(startingPosx, startingPosy, startingHeading);

        drive.setPoseEstimate(startingPosition);

        //trajectoryStorage goes xPos, yPos, heading then next trajectory starts

        ArrayList<Double> trajectoryStorage = new ArrayList<>();

        boolean a1Pressable = true;

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            Pose2d poseEstimate = drive.getPoseEstimate();

            drive.update();

            boolean a1 = this.gamepad1.a;

            if (a1 && a1Pressable) {
                trajectoryStorage.add(poseEstimate.getX());
                trajectoryStorage.add(poseEstimate.getY());
                trajectoryStorage.add(Math.toDegrees(poseEstimate.getHeading()));
            }

            a1Pressable = !a1;


            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Storage ", trajectoryStorage);
            telemetry.update();
        }
    }
}
