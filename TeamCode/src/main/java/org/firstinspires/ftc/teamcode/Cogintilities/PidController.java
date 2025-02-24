package org.firstinspires.ftc.teamcode.Cogintilities;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PidController {
    private final double proportionalGain; // kP
    private final double integralGain;     // kI
    private final double derivativeGain;   // kD
    private final double tolerance;
    private final double maxOutput;
    private final double minOutput;

    private double targetPosition;
    private double accumulatedError; // Integral
    private double previousError;
    private final ElapsedTime timer;
    private double lastUpdateTime;

    /**
     * Constructor for the PID controller.
     *
     * @param proportionalGain The proportional gain (kP).
     * @param integralGain     The integral gain (kI).
     * @param derivativeGain   The derivative gain (kD).
     * @param tolerance        The acceptable error range around the target.
     * @param maxOutput        The maximum output value.
     * @param minOutput        The minimum output value.
     */
    public PidController(double proportionalGain, double integralGain, double derivativeGain, double tolerance, double maxOutput, double minOutput) {
        this.proportionalGain = proportionalGain;
        this.integralGain = integralGain;
        this.derivativeGain = derivativeGain;
        this.tolerance = tolerance;
        this.maxOutput = maxOutput;
        this.minOutput = minOutput;

        this.targetPosition = 0.0;
        this.accumulatedError = 0.0;
        this.previousError = 0.0;
        this.timer = new ElapsedTime();
        this.lastUpdateTime = 0.0;
    }

    /**
     * Sets the target position for the PID controller.
     *
     * @param targetPosition The desired target position.
     */
    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
        resetIntegral();
        this.previousError = 0.0;
        timer.reset();
        this.lastUpdateTime = 0.0;
    }

    /**
     * Resets the integral term of the PID controller.
     */
    public void resetIntegral() {
        this.accumulatedError = 0.0;
    }

    /**
     * Updates the PID controller and calculates the output.
     *
     * @param currentPosition The current position of the system.
     * @return The calculated output value.
     */
    public double update(double currentPosition) {
        double error = targetPosition - currentPosition;
        double currentTime = timer.seconds();
        double deltaTime = currentTime - lastUpdateTime;
        lastUpdateTime = currentTime;

        // Avoid division by zero if deltaTime is zero
        if (deltaTime == 0) {
            deltaTime = 0.001; // Use a small value to avoid division by zero
        }

        // Proportional term
        double proportional = proportionalGain * error;

        // Integral term
        accumulatedError += error * deltaTime;
        double integralTerm = integralGain * accumulatedError;

        // Derivative term
        double derivative = derivativeGain * (error - previousError) / deltaTime;
        previousError = error;

        // Calculate the total output
        double output = proportional + integralTerm + derivative;

        // Limit the output to the allowed range
        output = Math.max(Math.min(output, maxOutput), minOutput);

        return output;
    }

    /**
     * Checks if the current position is within the tolerance of the target position.
     *
     * @param currentPosition The current position of the system.
     * @return True if the current position is at the target, false otherwise.
     */
    public boolean isAtTarget(double currentPosition) {
        return Math.abs(targetPosition - currentPosition) <= tolerance;
    }

    // Example Usage (in an FTC OpMode):

    //In your opmode initialization:
    // PIDController controller = new PIDController(kP, kI, kD, tolerance, maxOutput, minOutput);

    //In your opmode loop where you want to use the PID controller:

    /*
    double currentPosition = motor.getCurrentPosition(); //get current motor position
    double motorPower = controller.update(currentPosition);

    motor.setPower(motorPower); //apply power to motor

    if (controller.isAtTarget(currentPosition)){
        //logic for when the motor reaches the target.
        motor.setPower(0);
        telemetry.addData("Target Reached", currentPosition);
    }

    telemetry.addData("Current Position", currentPosition);
    telemetry.addData("Output power", motorPower);
    telemetry.update();
    */
}