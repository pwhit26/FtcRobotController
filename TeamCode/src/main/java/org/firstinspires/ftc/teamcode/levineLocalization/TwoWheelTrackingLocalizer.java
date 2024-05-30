package org.firstinspires.ftc.teamcode.levineLocalization;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunnerStuff.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->

 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.68897; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    //Normal Bot
    public static double PARALLEL_X = 0; // X is the up and down direction
    public static double PARALLEL_Y = -6.75; // Y is the strafe direction

    public static double PERPENDICULAR_X = -2;
    public static double PERPENDICULAR_Y = -1.75;

    //Freight Bot
//    public static double PARALLEL_X = -0.25; // X is the up and down direction
//    public static double PARALLEL_Y = -3; // Y is the strafe direction
////
//    public static double PERPENDICULAR_X = -4.25;
//    public static double PERPENDICULAR_Y = 0.5;

//    Multiplier for normal field
    public static double X_MULTIPLIER = 1.1045; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1.1045; // Multiplier in the Y direction

//    //Multiplier for foofy field (24 inch tiles)
//
//    public static double X_MULTIPLIER = 1.1275; // Multiplier in the X direction
//    public static double Y_MULTIPLIER = 1.1275; // Multiplier in the Y direction

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    public Encoder parallelEncoder, perpendicularEncoder;

    private SampleMecanumDrive drive;

    public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.drive = drive;

        //Normal bot
// Spencer is extra stinky and foofy and stinky and foofy and stinkey and foofy and stinky and foofy and stinky and foofy and stinkey and foofy and stinky and foofy and stinky and foofy and stinkey and foofy and stinky and foofy and stinky and foofy and stinkey and foofy and stinky and foofy and stinky and foofy and stinkey and foofy and stinky and foofy and stinky and foofy and stinkey and foofy and stinky and foofy and stinky and foofy and stinkey and foofy and stinky and foofy and stinky and foofy and stinkey and foofy and stinky and foofy and stinky and foofy and stinkey and foofy and stinky and foofy and stinky and foofy and stinkey and foofy and stinky and foofy and stinky and foofy and stinkey and foofy and stinky and foofy and stinky and foofy and stinkey and foofy and stinky and foofy and stinky and foofy and stinkey and foofy and stinky and foofy and stinky and foofy and stinkey and foofy and stinky and foofy and stinky and foofy and stinkey and foofy and stinky and foofy and stinky and foofy and stinkey and foofy and stinky and foofy and stinky and foofy and stinkey and foofy and stinky and foofy and stinky and foofy and stinkey and foofy and stinky and foofy and stinky and foofy and stinkey and foofy and stinky and foofy
        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontRight"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backLeft"));

        //Freight bot
//        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "parEncoder"));
//        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "perpEncoder"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)

        //Normal Bot
        perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);

        //Freight bot
//        parallelEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }
}