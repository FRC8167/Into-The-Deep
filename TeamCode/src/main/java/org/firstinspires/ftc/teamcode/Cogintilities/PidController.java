package org.firstinspires.ftc.teamcode.Cogintilities;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot.TeamConstants;

/**
 * A PID (Proportional-Integral-Derivative) controller implementation for controlling a system's position.
 * This class provides methods to set the target position, update the controller, and check if the system
 * has reached the target.
 */
public class PidController {
    double target;
    double error;
    double lastError;
    double power;
    double kP;
    double kD;
    double tolerance;
    Time time;

    public PidController(double kP, double kI, double kD, double tolerance) {
        time = new Time();

        error = 0;
        lastError = 0;
        this.tolerance = tolerance;

        this.kP = kP;
        this.kD = kD;
    }

    public double update(double pos){
        lastError = error;

        error = target - pos;

        power = Range.clip(error * kP + kD * (error - lastError) / time.seconds(), -1, 1);
        time.reset();
        return power;
    }
    public void setTarget(double target){
        this.target = target;
    }

    public double getTargetPosition(){
        return target;
    }

    public boolean isGood(){
        return (Math.abs(error) <= tolerance);
    }

}