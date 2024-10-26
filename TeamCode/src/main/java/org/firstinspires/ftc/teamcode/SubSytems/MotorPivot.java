package org.firstinspires.ftc.teamcode.SubSytems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * HINT: For running a motor to a specific target position, use the following procedure:
 *      - set target position tolerance (must be in encoder counts) this could be class level
 *        with an entry set in TeamConstants or set as a parameter for each move, your choice.
 *      - set target position (ensure system stays in range of allowable motion) (must be encoder counts)
 *      - set motor to RUNT_TO_POSITION mode
 *      - Command to position using setVelocity. Parameter is the maximum motor velocity you want
 *        the motor to use. Two methods are available, one uses a rate in [encoder counts/sec]
 *        and the other you can specify units of [degrees/sec] or [radians/sec].
 *
 *  Tune system to find a value of Kp (Proportional Coefficient in PID) and Kf (feed forward term
 *  if necessary) to achieve the target position in the least amount of time. See second link below
 *  for the functions used to set these values.
 *
 *  Create boolean function to let the outside world know if the motor is in motion to a target position.
 *  This will be used to prevent the operator from manually moving the system while it is
 *  already in motion to a target.  You can do this by checking if the current position is within
 *  the tolerance of the target or use the builtin isBusy() method.  One thing to keep in mind is what
 *  if something prevents the system from completing its motion to the target. This could result
 *  from poor tuning or damage occurred from colliding with another robot. Do you need a way to
 *  exit from run to position in this case?
 *
 * Info for motor caontrol and available motor functions can be found at the links below:
 *  https://docs.revrobotics.com/duo-control/programming/using-encoder-feedback#choosing-a-motor-mode
 *  https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/DcMotorEx.html
 */

public class MotorPivot {

    DcMotorEx motor;

    public MotorPivot(DcMotorEx motor){
        this.motor = motor;
    }

    public void periodic(){

    }

    public void setPosition(double degrees){
        /* Need to convert degrees to counts. You can then call setPosition(int counts) below */
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
