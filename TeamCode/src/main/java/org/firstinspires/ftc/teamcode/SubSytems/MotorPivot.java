package org.firstinspires.ftc.teamcode.SubSytems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MotorPivot {

    DcMotorEx motor;

    public MotorPivot(DcMotorEx motor){
        this.motor = motor;
    }

    public void periodic(){

    }

    public void setPosition(double degrees){

    }

    public void setPosition(int counts){

    }

    private void clamp(){

    }

    public int getPosition(){
        return(motor.getCurrentPosition());
    }

    public double getVelocity(){
        return(motor.getVelocity());
    }


}
