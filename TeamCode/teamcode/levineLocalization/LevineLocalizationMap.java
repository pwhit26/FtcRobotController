package org.firstinspires.ftc.teamcode.levineLocalization;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

//This is where wheel powers are set and constant for error and follow radius are set
@Config
public class LevineLocalizationMap {
    //Define all variables

    //Error of the robots position * 2
    public static double poseError = 1.5;

    //Angular error of the robot
    public static double angError = Math.toRadians(1);

    //Follow radius of the robot - smaller means robot follows points more directly, larger means robot will go around points in a more curved manner
    public static double followRadius = 5;

    //Increasing will create more in between points in PointFollower - vice versa for decrease
    public static double poseFollowCoef = 1;

    //Speed multiplier for wheel powers
    public static double speedMultiplier = 1;

    //Starting pose of robot
    public Pose2d startingPose;
    private LinearOpMode myOpMode;
    public DcMotor frontLeft, frontRight, backLeft, backRight;
    public VoltageSensor batteryVoltageSensor;

    //Constructor for class - define opmode to allow motors to be run from this file
    public LevineLocalizationMap(LinearOpMode opmode){
        myOpMode = opmode;
    }

    //Init code - uses init with default start of (0 - x, 0 - y, 90 degrees - heading)
    public void init() {
        init(new Pose2d(0, 0, Math.toRadians(90)));
    }

    //Init function - defines all devices and directions
    public void init(Pose2d startingPose) {
        //Define motors and directions and battery and starting pose
        frontLeft = myOpMode.hardwareMap.get(DcMotor.class, ("frontLeft")); //port 3
        frontRight = myOpMode.hardwareMap.get(DcMotor.class, ("frontRight")); //port 2
        backLeft = myOpMode.hardwareMap.get(DcMotor.class, ("backLeft")); //port 1
        backRight = myOpMode.hardwareMap.get(DcMotor.class, ("backRight"));  //port 0

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        batteryVoltageSensor = myOpMode.hardwareMap.voltageSensor.iterator().next();

        this.startingPose = startingPose;

        //Displays all telemetry data on FTC dashboard
        Telemetry telemetry = new MultipleTelemetry(this.myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());

        //Displays current battery voltage on FTC dashboard
        telemetry.addData("Battery Voltage: ", batteryVoltageSensor.getVoltage());
        telemetry.update();
    }

    //Sets all motor powers with the speed multiplier set here
    public void setMotorPowers(Pose2d currPose, Pose2d targetPose){
        // Get Powers
        WheelPowers powers = getPowers(currPose, targetPose);

        // Set powers
        frontLeft.setPower(powers.flp);
        frontRight.setPower(powers.frp);
        backLeft.setPower(powers.blp);
        backRight.setPower(powers.brp);
    }

    //Sets all motor powers based off position and allows for a speed multiplier
    public void setMotorPowers(Pose2d currPose, Pose2d targetPose, double speedMultiplier){
        // Get Powers
        WheelPowers powers = getPowers(currPose, targetPose, speedMultiplier);

        // Set powers
        frontLeft.setPower(powers.flp);
        frontRight.setPower(powers.frp);
        backLeft.setPower(powers.blp);
        backRight.setPower(powers.brp);
    }

    //Stops all motors
    public void stopMotors(){
        // Set powers
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    public WheelPowers getPowers(Pose2d currPose, Pose2d targetPose) {
        return getPowers(currPose, targetPose, speedMultiplier);
    }

    //Allows the robot to set each wheel power based off position and heading
    public WheelPowers getPowers(Pose2d currPose, Pose2d targetPose, double speedMultiplier) {
        // Get dists
        double xDist = targetPose.getX() - currPose.getX();
        double yDist = targetPose.getY() - currPose.getY();
        double totAngDist = targetPose.getHeading() - currPose.getHeading();

        // Get theta
        double theta = Math.atan2(yDist, xDist) + Math.toRadians(90) - targetPose.getHeading();

        // Get Powers
        double ADPower = speedMultiplier * (Math.sin(theta) + Math.cos(theta));
        double BCPower = speedMultiplier * (Math.sin(theta) - Math.cos(theta));

        double turnPower = MathsAndStuff.AngleWrap(totAngDist);
        double turningScale = Math.max(Math.abs(ADPower + turnPower), Math.abs(ADPower - turnPower));
        turningScale = Math.max(turningScale, Math.max(Math.abs(BCPower + turnPower), Math.abs(BCPower - turnPower)));

        if (Math.abs(turningScale) < 1.0){
            turningScale = 1.0;
        }

        double fl = (ADPower - turnPower) / turningScale;
        double fr = (BCPower + turnPower) / turningScale;
        double bl = (BCPower - turnPower) / turningScale;
        double br = (ADPower + turnPower) / turningScale;

        return new WheelPowers(fr, fl, br, bl);
    }
}
