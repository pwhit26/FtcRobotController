package org.firstinspires.ftc.teamcode.levineLocalization;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HaolinIsStupid;

public class ActionRunnerCenterStageAuton {
    public LinearOpMode myOpMode;
    HaolinIsStupid wBot;
    Telemetry telemetry;

    public ActionRunnerCenterStageAuton(LinearOpMode opMode, HaolinIsStupid wBot){
        myOpMode = opMode;
        this.wBot = wBot;
        telemetry = new MultipleTelemetry(this.myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    public void runActions(String action){

    }
}
