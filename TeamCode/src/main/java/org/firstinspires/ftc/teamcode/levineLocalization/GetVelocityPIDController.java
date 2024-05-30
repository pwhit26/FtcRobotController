package org.firstinspires.ftc.teamcode.levineLocalization;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

//This is used for the velocity control of the robot, allowing the wheel powers to be set to coorospond to the velocity
public class GetVelocityPIDController {
    private double kp; // Proportional gain
    private double ki; // Integral gain
    private double kd; // Derivative gain
    private double setpoint;
    private double integral;
    private double previousError;

    public GetVelocityPIDController(PIDCoefficients PIDv, double setpoint) {
        this.kp = PIDv.p;
        this.ki = PIDv.i;
        this.kd = PIDv.d;
        this.setpoint = setpoint;
        this.integral = 0;
        this.previousError = 0;
    }
    public void changeTarget(double newTarget){
        setpoint = newTarget;
    }

    public double calculate(double input) {
        double error = setpoint - input;

        double proportionalTerm = kp * error;

        integral += error;
        double integralTerm = ki * integral;

        double derivative = error - previousError;
        double derivativeTerm = kd * derivative;

        double output = proportionalTerm + integralTerm + derivativeTerm;

        previousError = error;

        return output;
    }
}