package org.firstinspires.ftc.teamcode.SubSytems;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoToggle extends Servo1D {


    public ServoToggle(Servo servo, double initPos, double min, double max) {
        super(servo, initPos, min, max);
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

    public void setToToggle(){
        toggleGripper();
        toggleGripper();
    }
    public void spinSpecimen(){
        setPosition(GRIPPER_CLOSE-0.025);
    }
    public void extraOpen(){
        setPosition(GRIPPER_OPEN-0.1);
    }

    /* ************************* Actions * *************************/
    public class Toggle implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            toggleGripper();
            return false;
        }
    }

    public Action toggle() {
        return new Toggle();
    }

    public class Spin implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            spinSpecimen();
            return false;
        }
    }

    public Action spin() {
        return new Spin();
    }


    public class EOpen implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            extraOpen();
            return false;
        }
    }

    public Action eOpen() {
        return new EOpen();
    }


}
