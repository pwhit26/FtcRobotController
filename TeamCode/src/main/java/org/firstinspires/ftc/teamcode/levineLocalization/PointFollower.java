package org.firstinspires.ftc.teamcode.levineLocalization;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

//This is Levine Localization, Point Follower, the math behind Levine Localization.

// Current Known Issues:
// Slow Down issue - When Robot is moving slower than the target vel and is in the range of decelerating, it will go slower than it is supposed
// This usually happens after turns
// Possibly solved by new target vel decel thing?
// Too small issue - when a path is too small for the Robot, it will cause some kind of error with list range
// Twitch issue - The robot is moving too fast and starts to twitch back and forth Making big strides but not too much of an issue
// Humping issue - The robot appears to be humping the ground - mostly solved by better an error and pose error, maybe also could be solved by slower slowest vel?

@Config
public class PointFollower {
    LinearOpMode myOpMode;
    LevineLocalizationMap wMap;
    Telemetry telemetry;
    public SampleMecanumDrive drive;
    ActionRunnerCenterStageAuton actionRunner;
    public ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime velTime = new ElapsedTime();
    public static double isBuggingRuntimeToStop;
    public static double isBuggingRuntimeToStopError = 2;
    public static double upperAngleToWrapAngError = 355, lowerAngleToWrapAngError = 5;
    int endOfPointCounter = 0;
    ArrayList <PosesAndActions> posesToGoTo = new ArrayList<>();
//    ArrayList<PointType> pointTypes = new ArrayList<>();
    ArrayList<Pose2d> inBetweenPoints = new ArrayList<>();
    ArrayList<PointType> pointTypesInBetween = new ArrayList<>();
    boolean xDone = false, yDone = false, angDone = false;
    int donePointCounter = 0;
    Pose2d startOfNewGo = new Pose2d(), prevPoseForVel = new Pose2d();
    public double currVelocity = 0;
    public double currPower;
    public static double maxVel = 50, testVel = 20, slowestVel = 2, roomForErrorVelocity = 20;
    public static double decceleration = 40;
    public double targetVelocity = maxVel;
    public static double accelerationConst = 200;
    public static PIDCoefficients PIDVals = new PIDCoefficients(0.25, 0, 0.5);
    public int isBuggingCounter = 0;

    //Constructor
    public PointFollower(LinearOpMode opmode, ActionRunnerCenterStageAuton actionRunner) {
        myOpMode = opmode;
        wMap = new LevineLocalizationMap(this.myOpMode);
        telemetry = new MultipleTelemetry(this.myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        this.actionRunner = actionRunner;
        targetVelocity = maxVel;
        telemetry.addData("Velocity ", currVelocity);
        telemetry.addData("targetVelocity ", targetVelocity);
        telemetry.update();
    }
    //Init - This is run the first time that point follower is run and is also run by reinit
    public void init(ArrayList<PosesAndActions> posesToGoTo, boolean isTest, boolean firstTime) {
        if(isTest){
            maxVel = testVel;
        }

        if(firstTime){
            drive = new SampleMecanumDrive(this.myOpMode.hardwareMap);
            posesToGoTo.add(0, posesToGoTo.get(0));
        }

        wMap.init(posesToGoTo.get(0).pose);
        startOfNewGo = posesToGoTo.get(0).pose;
        drive.setPoseEstimate(posesToGoTo.get(0).pose);

        this.posesToGoTo.clear();
//        pointTypes.clear();
        inBetweenPoints.clear();
        pointTypesInBetween.clear();

        this.posesToGoTo.addAll(posesToGoTo);

//        // Makes all points mid to make their pose error equal to follow radius
//        for (int i = 0; i < this.posesToGoTo.size(); i++) {
//            pointTypes.add(new PointType("mid"));
//        }
//        // This makes the last point an "end" point to allow it run actions
//        pointTypes.set(pointTypes.size() - 1, new PointType("end"));

        double theoreticalTheta;
        for (int i = 1; i < this.posesToGoTo.size(); i++) {
            Pose2d startingPos = this.posesToGoTo.get(i - 1).pose;
            Pose2d targetPos = this.posesToGoTo.get(i).pose;

            double xDist = targetPos.getX() - startingPos.getX();

            double yDist = targetPos.getY() - startingPos.getY();

            double totDistToTarget = Math.hypot(xDist, yDist);

            //Iteration for poses calculator
            int iterationsForPoses = (int) ((totDistToTarget / 2) * LevineLocalizationMap.poseFollowCoef);
            if(iterationsForPoses == 0){
                pointTypesInBetween.add(new PointType("mid"));
                inBetweenPoints.add(targetPos);
            }

            double distToTarget = totDistToTarget / iterationsForPoses;

            theoreticalTheta = MathsAndStuff.AngleWrap(Math.atan2(yDist, xDist));

            // Creates a bunch of points in between for two reasons:
            // 1: if the robot gets off course, it can get back on pretty easily
            // 2: in order for the robot to use its follow radius in order to make curves just like pure pursuit
            for (int j = 0; j < iterationsForPoses; j++) {
                double relDistX = Math.cos(theoreticalTheta) * (distToTarget * j);
                double relDistY = Math.sin(theoreticalTheta) * (distToTarget * j);
                inBetweenPoints.add(new Pose2d(startingPos.getX() + relDistX, startingPos.getY() + relDistY, targetPos.getHeading()));
            }

            //Makes all in between points mid
            for(int j = 0; j < iterationsForPoses; j++){
                pointTypesInBetween.add(new PointType("mid"));
            }
            //Makes last in between point "endofpoint" to run actions
            pointTypesInBetween.set(pointTypesInBetween.size() - 1, new PointType("endofpoint"));
        }
        // Adds target pose onto inbetween points
        inBetweenPoints.add(new Pose2d(this.posesToGoTo.get(this.posesToGoTo.size() - 1).pose.getX(), this.posesToGoTo.get(this.posesToGoTo.size() - 1).pose.getY(), this.posesToGoTo.get(this.posesToGoTo.size() - 1).pose.getHeading()));

        // Makes final point final so that the robot ends within pose error
        pointTypesInBetween.add(new PointType("final"));
        this.posesToGoTo.add(posesToGoTo.get(posesToGoTo.size()-1));

        telemetry.addLine("Poses to go to " + this.posesToGoTo);
        telemetry.addLine("Poses in between " + inBetweenPoints);
        telemetry.addLine("PointTypes in between: " + pointTypesInBetween);
        telemetry.update();
    }

    //reinit - This saves the last known pose as a pose in the next poses to go to
    public void reinit(ArrayList<PosesAndActions> posesToGoTo) {
        drive.update();
        posesToGoTo.add(0, new PosesAndActions(drive.getPoseEstimate(), ""));
        init(posesToGoTo, false, false);
    }

    //This makes the robot go to the points with the default max vel
    public void goToPoints(boolean stopAfter){
        goToPoints(stopAfter, maxVel);
    }

    //This makes the robot go to the points with a customizable max vel
    public void goToPoints(boolean stopAfter, double newMaxVel) {
        startOfNewGo = drive.getPoseEstimate();
        prevPoseForVel = drive.getPoseEstimate();
        currPower = 0;
        GetVelocityPIDController getVel = new GetVelocityPIDController(PIDVals, targetVelocity);
        velTime.reset();
        Pose2d currPose = drive.getPoseEstimate();
        targetVelocity = newMaxVel;
        double distNeededToStartDecel = ((Math.pow(slowestVel, 2) - Math.pow(newMaxVel, 2))/(-2*decceleration)) + (LevineLocalizationMap.followRadius/2);
        double totXDist = 0;
        double totYDist = 0;
        for (int i = 1; i < posesToGoTo.size(); i++) {
            totXDist += Math.abs(posesToGoTo.get(i).pose.getX() - posesToGoTo.get(i-1).pose.getX());
            totYDist += Math.abs(posesToGoTo.get(i).pose.getY() - posesToGoTo.get(i-1).pose.getY());
        }
        double totDistToTarget = Math.hypot(totXDist, totYDist);
        double prevDistToTarget = totDistToTarget;

        if(distNeededToStartDecel > totDistToTarget){
            targetVelocity = Math.sqrt(Math.abs(Math.pow(slowestVel, 2) + (2*decceleration*totDistToTarget)));
        }
        posesToGoTo.remove(0);

        while (!(inBetweenPoints.isEmpty())) {
            getVel.changeTarget(targetVelocity);
            drive.update();

            currPose = drive.getPoseEstimate();

            double timeForVel = velTime.seconds();

            double totDistForVel = Math.hypot(currPose.getX() - prevPoseForVel.getX(), currPose.getY() - prevPoseForVel.getY());
            currVelocity = Math.abs(totDistForVel / timeForVel);
            isBuggingRuntimeToStop = isBuggingRuntimeToStopError;

            if (!inBetweenPoints.isEmpty()) {
                prevDistToTarget = totDistToTarget;
                double isBuggingChecker = runtime.seconds();
                Pose2d targetPose = inBetweenPoints.get(0);
                double roomForPoseError = new PointType("mid").followRadius / 2;
                if (stopAfter){
                    roomForPoseError = pointTypesInBetween.get(0).followRadius / 2;
                }
                else{
                    roomForPoseError = new PointType("mid").followRadius / 2;
                }

                angDone = !pointTypesInBetween.get(0).type.equals("final");

                double angleForAngDist = currPose.getHeading();

                if(Math.toRadians(targetPose.getHeading()) > Math.toRadians(upperAngleToWrapAngError) && angleForAngDist < Math.toRadians(lowerAngleToWrapAngError)){
                    angleForAngDist += Math.toRadians(360);
                }
                else if(Math.toRadians(targetPose.getHeading()) < Math.toRadians(lowerAngleToWrapAngError) && angleForAngDist > Math.toRadians(upperAngleToWrapAngError)){
                    angleForAngDist -= Math.toRadians(360);
                }

                double xDist = targetPose.getX() - currPose.getX();
                double yDist = targetPose.getY() - currPose.getY();
                double angDist = targetPose.getHeading() - angleForAngDist;

                double distToTarget = Math.hypot(xDist, yDist);
                double theta = MathsAndStuff.AngleWrap(Math.atan2(xDist, yDist) + wMap.startingPose.getHeading());

                if(posesToGoTo.size()-1 >= 0){
                    totXDist = Math.abs(posesToGoTo.get(0).pose.getX() - currPose.getX());
                    totYDist = Math.abs(posesToGoTo.get(0).pose.getY() - currPose.getY());
                    if(posesToGoTo.size() > 2){
                        for (int i = 2; i < posesToGoTo.size(); i++) {
                            totXDist += Math.abs(posesToGoTo.get(i).pose.getX() - posesToGoTo.get(i-1).pose.getX());
                            totYDist += Math.abs(posesToGoTo.get(i).pose.getY() - posesToGoTo.get(i-1).pose.getY());
                        }
                    }
                }

                totDistToTarget = Math.hypot(totXDist, totYDist);

                //Error X
                double relDistX = Math.cos(theta) * distToTarget;
                double relErrorX = (Math.cos(theta) * roomForPoseError);

                double relDistY = Math.sin(theta) * distToTarget;
                double relErrorY = (Math.sin(theta) * roomForPoseError);

                if (Math.abs(relDistX) < Math.abs(relErrorX)) {
                    xDone = true;
                }
                if (Math.abs(relDistY) < Math.abs(relErrorY)) {
                    yDone = true;
                }

                if (stopAfter){
//                    if(Math.abs(totDistToTarget) < Math.abs(distNeededToStartDecel) && totDistToTarget < prevDistToTarget + targetVelocity*timeForVel && targetVelocity >= currVelocity - roomForErrorVelocity){
//                        targetVelocity = targetVelocity - (decceleration*timeForVel);
//                    }
                    if(Math.abs(totDistToTarget) <= Math.abs(distNeededToStartDecel)){
                        double coolMath = Math.pow(slowestVel, 2) + (2*decceleration*(totDistToTarget - roomForPoseError));
                        if(coolMath > 0){
                            targetVelocity = Math.sqrt(Math.pow(slowestVel, 2) + (2*decceleration*(totDistToTarget - roomForPoseError)));
                        }
                        else{
                            targetVelocity = slowestVel;
                        }

                    }
//                    else if(Math.abs(totDistToTarget) > Math.abs(distNeededToStartDecel)){
//                        targetVelocity = newMaxVel;
//                    }
                    else{
                        targetVelocity = newMaxVel;
                    }
                }
                else{
                    targetVelocity = newMaxVel;
                }
                if(targetVelocity < slowestVel){
                    targetVelocity = slowestVel;
                }
                if (currVelocity < targetVelocity || currVelocity > targetVelocity) {
                    currPower += getVel.calculate(currVelocity) / accelerationConst;
                }
                if (currPower < 0) {
                    currPower = 0;
                }
                if (currPower > 1) {
                    currPower = 1;
                }
//                currPower = 0;

                wMap.setMotorPowers(currPose, targetPose, currPower);

                if (isBuggingChecker > isBuggingRuntimeToStop) {
                    xDone = true;
                    yDone = true;
                    angDone = true;
                    isBuggingCounter++;
                }
                if (Math.abs(angDist) < Math.abs(LevineLocalizationMap.angError)) {
                    angDone = true;
                }

                prevPoseForVel = currPose;
                velTime.reset();

                if (xDone && yDone && angDone) {
                    if(pointTypesInBetween.get(0).type.equals("endofpoint")){
                        //Normally actionRunner should be used

                        //Normal Bot
                        actionRunner.runActions(posesToGoTo.get(0).action);

                        //Freight Bot
//                        actionRunner2.runActions(posesToGoTo.get(0).action);
                        posesToGoTo.remove(0);
                        endOfPointCounter++;
                    }

                    runtime.reset();
                    inBetweenPoints.remove(0);
                    pointTypesInBetween.remove(0);

                    donePointCounter++;
                    xDone = false;
                    yDone = false;
                    startOfNewGo = new Pose2d(targetPose.getX(), targetPose.getY(), targetPose.getHeading());
                }

                telemetry.addData("Velocity ", currVelocity);
                telemetry.addData("totXDist ", totXDist);
                telemetry.addData("totYDist ", totYDist);
                telemetry.addData("Pose error ", roomForPoseError);
                telemetry.addData("targetVelocity ", targetVelocity);
                telemetry.addData("New Max Vel ", newMaxVel);
                telemetry.addData("Tot dist to target ", totDistToTarget);
                telemetry.addData("Ang Dist ", angDist);
                telemetry.addData("posesToGoTo ", posesToGoTo);
                telemetry.addData("currPower ", currPower);
                telemetry.addLine("isBuggingChecker: " + isBuggingChecker);
                telemetry.addData("DistNeededToStartDecel ", distNeededToStartDecel);
                telemetry.update();
            }
        }
        if(stopAfter) {
            wMap.stopMotors();
        }
    }
}