package org.firstinspires.ftc.teamcode.SubSytems;

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


    public void toggleGripper(){

        switch(state) {
            case OPEN:
                state = State.CLOSE;
                setPosition(GRIPPER_CLOSE);
                break;
            case CLOSE:
                state = State.OPEN;
                setPosition(GRIPPER_OPEN);
                break;
        }
    }

    public double servoPos() { return servo.getPosition(); }


}
