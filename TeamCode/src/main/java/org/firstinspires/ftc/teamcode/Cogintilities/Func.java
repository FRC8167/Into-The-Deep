package org.firstinspires.ftc.teamcode.Cogintilities;

import org.firstinspires.ftc.teamcode.Robot.TeamConstants;

/*
 *  Tune system to find a value of Kp (Proportional Coefficient in PID) and Kf (feed forward term
 *  if necessary) to achieve the target position in the least amount of time. See second link below
 *  for the functions used to set these values.
 *
 * Info for motor control and available motor functions can be found at the links below:
 *  https://docs.revrobotics.com/duo-control/programming/using-encoder-feedback#choosing-a-motor-mode
 *  https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/DcMotorEx.html
 */

public class Func implements TeamConstants {


    public double TriClampX(double x, double y) {
        double wristX = x;
        double wristY = y;
        double returnWristX = wristX;
        double returnWristY = wristY;

        if (Math.sqrt(wristX*wristX+wristY*wristY) < (408/25.4)){
            returnWristX = wristX/(Math.sqrt(wristX*wristX+wristY*wristY)/(408/25.4));
            returnWristY = wristY/(Math.sqrt(wristX*wristX+wristY*wristY)/(408/25.4));
        }

        wristX = returnWristX;
        wristY = returnWristY;
        if (Math.sqrt(wristX*wristX+wristY*wristY) > (TeamConstants.SLIDE_MAX*TeamConstants.INCHES_PER_COUNT+(408/25.4))){
            returnWristX = wristX/(Math.sqrt(wristX*wristX+wristY*wristY)/(TeamConstants.SLIDE_MAX*TeamConstants.INCHES_PER_COUNT+(408/25.4)));
            returnWristY = wristY/(Math.sqrt(wristX*wristX+wristY*wristY)/(TeamConstants.SLIDE_MAX*TeamConstants.INCHES_PER_COUNT+(408/25.4)));
        }

        wristX = returnWristX;
        wristY = returnWristY;
        if (wristX > TeamConstants.Wrist_X_MAX){
            returnWristX = TeamConstants.Wrist_X_MAX;
            returnWristY = wristY;
        }

        wristX = returnWristX;
        wristY = returnWristY;
        if (wristX < TeamConstants.Wrist_X_MIN){
            returnWristX = TeamConstants.Wrist_X_MAX;
            returnWristY = wristY;
        }

        wristX = returnWristX;
        wristY = returnWristY;
        return wristX;

    }


    public double TriClampY(double x, double y) {
        double wristX = x;
        double wristY = y;
        double returnWristX = wristX;
        double returnWristY = wristY;

        if (Math.sqrt(wristX*wristX+wristY*wristY) < (408 / 25.4)){
            returnWristX = wristX/(Math.sqrt(wristX*wristX+wristY*wristY)/(408 / 25.4));
            returnWristY = wristY/(Math.sqrt(wristX*wristX+wristY*wristY)/(408 / 25.4));
        }

        wristX = returnWristX;
        wristY = returnWristY;
        if (Math.sqrt(wristX*wristX+wristY*wristY) > (TeamConstants.SLIDE_MAX * TeamConstants.INCHES_PER_COUNT+(408/25.4))){
            returnWristX = wristX/(Math.sqrt(wristX*wristX+wristY*wristY)/(TeamConstants.SLIDE_MAX*TeamConstants.INCHES_PER_COUNT+(408 / 25.4)));
            returnWristY = wristY/(Math.sqrt(wristX*wristX+wristY*wristY)/(TeamConstants.SLIDE_MAX*TeamConstants.INCHES_PER_COUNT+(408 / 25.4)));
        }

        wristX = returnWristX;
        wristY = returnWristY;
        if (wristY < TeamConstants.Wrist_Y_MIN){
            returnWristX = wristX;
            returnWristY = TeamConstants.Wrist_Y_MIN;
        }

        wristX = returnWristX;
        wristY = returnWristY;

        return wristY;

    }


    public double TestNewX(double x,double y, double newx, double newy){

        if ((Math.toDegrees(-Math.atan2(newx, newy)) + 135) < 45){
            return x;
        }
        else {
            return newx;
        }

    }


    public double TestNewY(double x,double y, double newx, double newy){

        if ((Math.toDegrees(-Math.atan2(newx, newy)) + 135.0) < 45){
            return y;
        }
        else {
            return newy;
        }

    }



}
