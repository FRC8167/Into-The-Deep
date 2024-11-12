package org.firstinspires.ftc.teamcode.SubSytems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot.TeamConstants;

public class Slide implements TeamConstants {

    DcMotorEx slideMotor;
    int min;
    double max;


    public Slide(DcMotorEx slideMotor, double initPos, int minCounts, double max) {
        this.slideMotor = slideMotor;
        setPosition(initPos);
        this.min = minCounts;
        this.max = max;
    }

    public void manualMove(double joystickValue) {
        setPositionCounts((int)(slideMotor.getCurrentPosition() + 40 * joystickValue));
    } //change to trigger button

    public void setPositionCounts(int counts){
        slideMotor.setTargetPosition(clamp(counts));
        slideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slideMotor.setVelocity(3000);
    }

    private int clamp(int position){
        if (position > MAX_POSITION_COUNTS){
            return MAX_POSITION_COUNTS;
        } else if (position < min){
            return min;
        }
        else return position;
    }

    public void setPosition(double position) {slideMotor.setPosition(Range.clip(position, min, max));
    }


    public double servoPos() { return slideMotor.getPosition(); }


    /** Action Classes **/
    public class SetServoPosition implements Action {

        double position;

        public SetServoPosition(double pos) {
            position = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            slideMotor.setPosition(position);
            return false;
        }

    }


    public Action setServoPosition(double position) {
        return new SetServoPosition(position);
    }

}
