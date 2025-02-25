package org.firstinspires.ftc.teamcode.Cogintilities;

public class Time {
    private double time;
    private double lastTime;

    public Time() {
        this.time = 0.0;
        this.lastTime = 0.0;
    }



    public double seconds() {
        time = System.currentTimeMillis() / 1000.0;
        return time-lastTime;
    }


    public void reset() {
        time = System.currentTimeMillis() / 1000.0;
        lastTime = time;
    }


}