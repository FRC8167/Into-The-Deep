package org.firstinspires.ftc.teamcode.SubSytems;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Robot.TeamConstants;

public class ServoPivot extends Servo1D {

    double angle = 45;
    double distFromGround = 0;
    double servoPos = 0;


    public ServoPivot(Servo servo, double init, double min, double max, boolean moveOnInit) {
        super(servo, init , min, max, moveOnInit);
    }


    public void setServoAngToGround(double targetAng, double armAng){
        servoPos= (90 - armAng + targetAng) / 270;
        if (servoPos >= 0){
            servo.setPosition(servoPos);
        }
        else {
            servo.setPosition(0);
        }
    }


    public boolean moveByPosFlat(double x,double y, boolean forward) {
        angle = ((180 - (Math.toDegrees(Math.atan2(x, y)))));
        distFromGround = 380.09193 / 25.4 + y - 3.37;
        setServoAngToGround(90, angle);
        return forward;
    }
    public boolean moveByPos90(double x,double y, boolean forward) {
        angle = ((180 - (Math.toDegrees(Math.atan2(x, y)))));
        distFromGround = 380.09193 / 25.4 + y - 3.37;
        setServoAngToGround(0, angle);
        return forward;
    }
    public boolean moveByPos(double x,double y, boolean forward){
        angle = ((180-(Math.toDegrees(Math.atan2(x, y)))));
        distFromGround = 380.09193/25.4 + y - 3.37;
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
    public void disable(){
        ((ServoImplEx)servo).setPwmDisable();
    }
    public void enable(){
        ((ServoImplEx)servo).setPwmEnable();
    }
    public boolean isEnabled(){
       return ((ServoImplEx)servo).isPwmEnabled();
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
    public class WristTrigFlat implements Action {

        public double newx;
        public double newy;
        public boolean newforward;

        public WristTrigFlat(double x, double y, boolean forward){newx = x; newy = y; newforward = forward;}

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            moveByPosFlat(newx, newy, newforward);
            return false;
        }

    }
    public Action wristTrigFlat(double x, double y, boolean forward) {
        return new WristTrigFlat(x,y,forward);
    }
    public class WristTrig90 implements Action {

        public double newx;
        public double newy;
        public boolean newforward;

        public WristTrig90(double x, double y, boolean forward){newx = x; newy = y; newforward = forward;}

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            moveByPos90(newx, newy, newforward);
            return false;
        }

    }
    public Action wristTrig90(double x, double y, boolean forward) {
        return new WristTrig90(x,y,forward);
    }


}
