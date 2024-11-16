package org.firstinspires.ftc.teamcode.SubSytems;


import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.TeamConstants;

public class ServoRotate extends Servo1D {
    double RotateAcuteAng;

    public ServoRotate(Servo servo, double initPos, double min, double max) {
        super(servo, initPos, min, max);
    }


    public double moveTrig(double joyX,double joyY){

        RotateAcuteAng = Math.abs(Math.toDegrees(Math.atan2(-1*joyY, joyX)));
        if (joyY == 0 && joyX == 0) setPosition(TeamConstants.WRIST_ROTATE_CENTER);
        else if (joyY<= 0) setPosition(((((RotateAcuteAng)/(300))+.2)));
        return RotateAcuteAng;

    }



}
