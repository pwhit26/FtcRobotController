package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class DriveRobot extends LinearOpMode
{
    public HaolinIsStupid bot = new HaolinIsStupid(this);

    public MecanumDrive drive;
    public static double movementSpeed=0.5;



    @Override
    public void runOpMode() throws InterruptedException
    {
        bot.init();
        drive=new MecanumDrive(bot.frontLeft, bot.frontRight, bot.backLeft, bot.backRight);
        waitForStart();
        while(opModeIsActive())
        {
            double ly1=gamepad1.left_stick_y;
            double lx1=gamepad1.left_stick_x;
            double rx1=gamepad1.right_stick_x;
            drive.moveInTeleop(lx1, ly1, rx1, movementSpeed);
        }
    }
}
