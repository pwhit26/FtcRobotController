package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class HaolinIsStupid
{
    public DcMotor  frontLeft, frontRight, backLeft, backRight;
    public LinearOpMode myOpMode;

    public HaolinIsStupid(LinearOpMode opMode)
    {
        myOpMode=opMode;
    }
    public void init()
    {
        frontLeft=myOpMode.hardwareMap.get(DcMotor.class,"frontLeft");

        frontRight=myOpMode.hardwareMap.get(DcMotor.class, "frontRight");
        backRight=myOpMode.hardwareMap.get(DcMotor.class, "backRight");
        backLeft=myOpMode.hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
