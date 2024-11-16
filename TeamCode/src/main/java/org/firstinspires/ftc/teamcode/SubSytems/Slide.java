package org.firstinspires.ftc.teamcode.SubSytems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot.TeamConstants;

public class Slide implements TeamConstants {

    DcMotorEx motor;
    int tolerance = 6;
    int minSlideCounts;
    int min;
    double max;


    public Slide(DcMotorEx motor) {
        this.motor = motor;
        motor.setTargetPositionTolerance(tolerance);
    }


    public void manualMove(double triggerValue) {
        setPositionCounts((int)(motor.getCurrentPosition() + 40 * triggerValue));
    }
    //TODO:  want to do this with a Trigger; figure this out in TeleOp??


    public void setPosition(double inches) {
        setPositionCounts(inchesToCounts(inches));
    }


    public void setPositionCounts(int counts){
        motor.setTargetPosition(clamp(counts));
        motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motor.setVelocity(3000);
    }


    private int clamp(int position){
        if (position > SLIDE_MAX){
            return SLIDE_MAX;
        } else if (position < SLIDE_MIN){
            return SLIDE_MIN;
        }
        else return position;
    }


    public int inchesToCounts(double inches){
        return (int)(inches * TeamConstants.INCHES_TO_COUNTS);
    }


    public double countsToInches(double counts) {
        return (counts * 1/TeamConstants.INCHES_TO_COUNTS);
    }


    /* ************************* Actions * *************************/
    public class SlidePosition implements Action {

        int position;

        public SlidePosition(int pos) {
            position = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setPositionCounts(position);
            return false;
        }

    }


    public Action rotateToPosition(int position) {
        return new SlidePosition(position);
    }
}
