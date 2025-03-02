package org.firstinspires.ftc.teamcode.Cogintilities;

public class LPF_basic {

    protected double alpha = 0.1;   // higher value increases filtering
    double previousValue = 0;

    /**
     *
     * @param alphaValue Filter Gain. (0 < alpha < 1) Higher value increases filtering.
     */
    public LPF_basic(double alphaValue) {
        alpha = alphaValue;
    }


    /************************************************************************
     * Low Pass Filter estimate
     * @param  rawValue measurement sensor signal
     * @return filtered value
     */
    public double filtered(double rawValue) {

        double filteredValue = alpha * previousValue + (1 - alpha) * rawValue;
        previousValue = filteredValue;

        return filteredValue;
    }

}
