package org.firstinspires.ftc.teamcode.SubSytems;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Robot.TeamConstants;

public class ServoRotate extends Servo1D {

    double rotateAcuteAng;

    public ServoRotate(Servo servo, double initPos, double min, double max, boolean moveOnInit) {
        super(servo, initPos, min, max, moveOnInit);
    }


    public void moveTrig(double joyX, double joyY) {
        rotateAcuteAng = Math.abs(Math.toDegrees(Math.atan2(-1 * joyY, joyX))); // 0 Right 180 Left
        if (joyY == 0 && joyX == 0)
            setPosition(TeamConstants.WRIST_ROTATE_CENTER);
        else if (joyY <= 0)
            setPosition(rotateAcuteAng / 300 + 0.2);
    }
    public void moveAng(double inAng){
        setPosition(inAng / 300 + 0.2);
    }


    public double getRotateAcuteAng() {
        return rotateAcuteAng;
    }

    public void disable(){
        ((ServoImplEx)servo).setPwmDisable();
    }
    public void enable(){
        ((ServoImplEx)servo).setPwmEnable();
    }

    public class RotateTrig implements Action {

        public double newAngle;
        public RotateTrig(double angle) {newAngle = angle;}
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            moveAng(newAngle);
            return false;
        }

    }


    public Action rotateTrig(double inputAng) {
        return new ServoRotate.RotateTrig(inputAng);


    }
}