package org.firstinspires.ftc.teamcode.SubSytems;

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


    int tolerance = 20;
    int minRotationCounts;
    double y = 161.7 / 25.4;        // Distance from wrist pivot joint to the floor
    double h = (336 + 48) / 25.4;   // Distance from arm pivot axis to the floor
    //double lmin = 408 / 25.4;
    //initialize position = 45; degrees;

    public double TriClampX(double x, double y) {
        double wristX = x;
        double wristY = y;
        if (Math.sqrt(wristX*wristX+wristY*wristY) < (408/25.4)){
            wristX = wristX/(Math.sqrt(wristX*wristX+wristY*wristY)/(408/25.4));
            wristY = wristY/(Math.sqrt(wristX*wristX+wristY*wristY)/(408/25.4));
        }
        if (Math.sqrt(wristX*wristX+wristY*wristY) > (TeamConstants.SLIDE_MAX*TeamConstants.INCHES_PER_COUNT+(408/25.4))){
            wristX = wristX/(Math.sqrt(wristX*wristX+wristY*wristY)/(TeamConstants.SLIDE_MAX*TeamConstants.INCHES_PER_COUNT+(408/25.4)));
            wristY = wristY/(Math.sqrt(wristX*wristX+wristY*wristY)/(TeamConstants.SLIDE_MAX*TeamConstants.INCHES_PER_COUNT+(408/25.4)));
        }

        return wristX;

    }
    public double TriClampY(double x, double y) {
        double wristX = x;
        double wristY = y;
        if (Math.sqrt(wristX*wristX+wristY*wristY) < (408/25.4)){
            wristX = wristX/(Math.sqrt(wristX*wristX+wristY*wristY)/(408/25.4));
            wristY = wristY/(Math.sqrt(wristX*wristX+wristY*wristY)/(408/25.4));
        }
        if (Math.sqrt(wristX*wristX+wristY*wristY) > (TeamConstants.SLIDE_MAX*TeamConstants.INCHES_PER_COUNT+(408/25.4))){
            wristX = wristX/(Math.sqrt(wristX*wristX+wristY*wristY)/(TeamConstants.SLIDE_MAX*TeamConstants.INCHES_PER_COUNT+(408/25.4)));
            wristY = wristY/(Math.sqrt(wristX*wristX+wristY*wristY)/(TeamConstants.SLIDE_MAX*TeamConstants.INCHES_PER_COUNT+(408/25.4)));
        }
        return wristY;

    }
    public double TestNewX(double x,double y, double newx, double newy){

        if ((Math.toDegrees(-Math.atan2(newx, newy))+135) < 45){
            return x;
        }
        else {
            return newx;
        }

    }
    public double TestNewY(double x,double y, double newx, double newy){

        if ((Math.toDegrees(-Math.atan2(newx, newy))+135) < 45){
            return y;
        }
        else {
            return newy;
        }

    }



}
