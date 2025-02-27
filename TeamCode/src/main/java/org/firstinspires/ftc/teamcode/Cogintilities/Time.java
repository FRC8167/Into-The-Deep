package org.firstinspires.ftc.teamcode.Cogintilities;

public class Time {
    private double time;
    private double lastTime;

    public Time() {
        this.time = System.currentTimeMillis();
        this.lastTime = System.currentTimeMillis();
    }



    public double seconds() {
        time = System.currentTimeMillis() / 1000.0;
        return time-lastTime;
    }

    public double milliseconds() {
        time = System.currentTimeMillis() / 1000.0;
        return (time-lastTime) * 1000;
    }

    public double minutes() {
        time = System.currentTimeMillis() / 1000.0;
        return (time - lastTime) / 60;
    }


    public void reset() {
        time = System.currentTimeMillis() / 1000.0;
        lastTime = time;
    }


}