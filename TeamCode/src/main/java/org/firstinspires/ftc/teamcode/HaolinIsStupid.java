package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

public class HaolinIsStupid
{
    public DcMotor  frontLeft, frontRight, backLeft, backRight;
    public Servo quincysThing;
    public LinearOpMode myOpMode;
    public HuskyLens huskyLens;
    private final int READ_PERIOD = 1;

    public HaolinIsStupid(LinearOpMode opMode)
    {
        myOpMode=opMode;
    }
    public void init()
    {
        frontLeft=myOpMode.hardwareMap.get(DcMotor.class,"frontLeft");
        frontRight = myOpMode.hardwareMap.get(DcMotor.class, "frontRight");
        backRight = myOpMode.hardwareMap.get(DcMotor.class, "backRight");
        backLeft = myOpMode.hardwareMap.get(DcMotor.class, "backLeft");
        quincysThing = myOpMode.hardwareMap.get(Servo.class, "Servo1");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        huskyLens = myOpMode.hardwareMap.get(HuskyLens.class, "huskyLens");
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
