package org.firstinspires.ftc.teamcode.Utils;

public class PIDController {
    double kP;
    double kI;
    double kD;
    double kDT;

    private double previousError = 0.0;
    private double integral = 0.0;

    public PIDController(double p, double i, double d, double dt) {
        kP = p;
        kI = i;
        kD = d;
        kDT = dt;
    }

    public double Calculate(double reference, double state) {
        double error = reference - state;
        integral += error * kDT;
        double derivative = (error - previousError) / kDT;
        double output = kP * error + kI * integral + kD * derivative;

        previousError = error;

        return output;
    }
}
