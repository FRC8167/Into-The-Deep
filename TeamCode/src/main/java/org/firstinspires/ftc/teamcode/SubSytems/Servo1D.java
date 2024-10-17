package org.firstinspires.ftc.teamcode.SubSytems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot.TeamConstants;

public class Servo1D implements TeamConstants {
    Servo servo;
    enum State{OPEN, CLOSE};
    State state;

    public Servo1D(Servo servo){
        this.servo = servo;
        state = State.CLOSE;
        setPosition(GRIPPER_CLOSE);
    };

    public void setPosition(double Position){
        servo.setPosition(Range.clip(Position, MIN_WRIST, MAX_WRIST));
    }

    public void toggleGripper(){
        switch(state){
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
}
