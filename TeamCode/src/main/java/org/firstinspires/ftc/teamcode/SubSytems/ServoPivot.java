package org.firstinspires.ftc.teamcode.SubSytems;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.TeamConstants;

public class ServoPivot extends Servo1D {

    double angle = 45;
    double distFromGround = 0;
    double servoPos = 0;


    public ServoPivot(Servo servo, double initPos, double min, double max) {
        super(servo, initPos, min, max);
    }


    public void setServoAngToGround(double targetAng, double armAng){
        servoPos= (90 - armAng + targetAng) / 240;
        if (servoPos >= 0){
            servo.setPosition(servoPos);
        }
        else {
            servo.setPosition(0);
        }
    }


    public boolean moveByPos(double x,double y, boolean forward){
        angle = ((180-(Math.toDegrees(Math.atan2(x, y)))))-7;
        distFromGround = 380.09193/25.4 + y- 3.37;
        if (angle <= 65){
            setPosition(TeamConstants.WRIST_PIVOT_MAX);
            return true;
        } else if (angle <= 90) {
            setServoAngToGround(0, angle);
            return true;
        } else if (y<= 17) {
            setServoAngToGround(90, angle);
            return true;
        } else if (angle < 170) {
            setServoAngToGround(90, angle);
            return true;
        } else if (170 <= angle && angle <= 190) {
            if (forward){
                setServoAngToGround(90, angle);

            }
            else{
                setServoAngToGround(180, angle);
            }
            return forward;
        } else if (angle <270) {
            setServoAngToGround(180, angle);
            return false;
        } else {
            setServoAngToGround(270, angle);
            return false;
        }

    }


    public double getAngle(){
        return angle;
    }


    public double getServoPos() {
            return servoPos*240;
    }


    public class WristTrig implements Action {

        public double newx;
        public double newy;
        public boolean newforward;

        public WristTrig(double x, double y, boolean forward){newx = x; newy = y; newforward = forward;}

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            moveByPos(newx, newy, newforward);
            return false;
        }

    }


    public Action wristTrig(double x, double y, boolean forward) {
        return new WristTrig(x,y,forward);
    }
}
