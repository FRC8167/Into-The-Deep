package org.firstinspires.ftc.teamcode.SubSytems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot.TeamConstants;

public class Servo1D implements TeamConstants {

    Servo servo;
    enum State{OPEN, CLOSE};
    State state;
    double min;
    double max;

    public Servo1D(Servo servo, double initPos, double min, double max) {
        this.servo = servo;
        state = State.CLOSE;
        setPosition(initPos);
        this.min = min;
        this.max = max;
    }

    public void setPosition(double position) {
        servo.setPosition(Range.clip(position, min, max));
    }

    public double servoPos() { return servo.getPosition(); }
    public class SetServoPosition implements Action {

        double position;


        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            servo.setPosition(position);
            return false;
        }


    }
    public Action setServoPosition(double position) {
        return new SetServoPosition();
    }
}
