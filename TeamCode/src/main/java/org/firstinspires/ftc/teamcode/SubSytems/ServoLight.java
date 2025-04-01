package org.firstinspires.ftc.teamcode.SubSytems;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.TeamConstants;

public class ServoLight extends Servo1D {

    double rotateAcuteAng;

    public ServoLight(Servo servo, double initPos, double min, double max) {
        super(servo, initPos, min, max);
    }

    public void setBrightness(double value) {
        setPosition(value);
    }

    public double getBrightness() {
        return servoPos();
    }
}