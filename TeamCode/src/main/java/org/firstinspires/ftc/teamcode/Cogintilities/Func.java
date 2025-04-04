package org.firstinspires.ftc.teamcode.Cogintilities;

import org.firstinspires.ftc.teamcode.Robot.RobotConfiguration;
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
            returnWristX = TeamConstants.Wrist_X_MIN;
            returnWristY = wristY;
        }
        wristX = returnWristX;
        wristY = returnWristY;
        return wristX;

    }
    public double clamp(double min, double val, double max){
        if (val > max){
            return max;
        } else if (val < min){
            return min;
        }
        else return val;
    }

    public double getServoAngToGround(double targetAng, double armAng){
        double servoPos= (90 - armAng + targetAng) / 270;
        if (servoPos >= 0){
            return (servoPos);
        }
        else {
            return(0);
        }
    }


    public double getMoveByPos(double x,double y){
        boolean forward = true;
        double angle = ((180-(Math.toDegrees(Math.atan2(x, y)))));
        double distFromGround = 380.09193/25.4 + y - 3.37;
        if (angle <= 65){
            return TeamConstants.WRIST_PIVOT_MAX;
        } else if (angle <= 90) {
            return getServoAngToGround(0, angle);

        } else if (y<= 17) {
            return getServoAngToGround(90, angle);
        } else if (angle < 170) {
            return getServoAngToGround(90, angle);

        } else if (170 <= angle && angle <= 190) {
            if (forward){
                return getServoAngToGround(90, angle);

            }
            else{
                return getServoAngToGround(180, angle);
            }


        } else if (angle <270) {
            return getServoAngToGround(180, angle);
        } else {
            return getServoAngToGround(270, angle);
        }

    }


    public double TriClampY(double x, double y) {
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

    public double reverseTrigX(double armPivotDeg, double slideIn){
        return Math.cos(Math.toRadians(armPivotDeg - 45)) * slideIn;
    }
    public double reverseTrigY(double armPivotDeg, double slideIn){
        return Math.sin(Math.toRadians(armPivotDeg - 45)) * slideIn;
    }



}
