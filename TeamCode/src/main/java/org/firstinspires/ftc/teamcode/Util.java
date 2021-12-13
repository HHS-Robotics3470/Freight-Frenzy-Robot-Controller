package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * contains a bunch of helpful static methods
 */
public class Util {
    /**
     * runs a given motor (that has an encoder) to a given position, at a given power
     * @param motor the motor to move
     * @param targetPosition    the position to move to
     * @param power the power to move at (must be positive)
     */
    public static void runMotorToPosition(DcMotor motor, int targetPosition, double power) {
        power = Math.abs(power);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set target
        motor.setTargetPosition(targetPosition);

        //set to RUN_TO_POSITION
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set power
        motor.setPower(power);
        //if (targetPosition > motor.getCurrentPosition()) motor.setPower(power);
        //else if (targetPosition < motor.getCurrentPosition()) motor.setPower(-power);

        //wait
        while (motor.isBusy()) {} //let the motor run to that position

        //stop, and go back to normal drive mode
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * given an array of rectangular coordinates (x,y), returns array of equivalent polar coordinates (r, theta)
     * @param rectangularCoords array of length 2 containing rectangular (x,y) coordinates
     * @return array of length 2 containing equivalent polar coordinates (or null values if passed array is too big / too small)
     */
    public static double[] convertRectangularToPolar(double[] rectangularCoords) {
        //error prevention
        if (rectangularCoords.length != 2) return new double[2];
        else {
            //calculations
            double x = rectangularCoords[0];
            double y = rectangularCoords[1];
            double radius   = Math.sqrt((x * x) + (y * y));    //radius, distance formula
            double theta    = Math.atan2(y, x);                //theta, arctangent(y/x)

            return new double[]{radius,theta};
        }
    }

    /**
     * given rectangular coordinates (x,y), returns array of equivalent polar coordinates (r, theta)
     * @param x x-coordinate
     * @param y y-coordinate
     * @return array of length 2 containing equivalent polar coordinates
     */

    public static double[] convertRectangularToPolar(double x, double y) {
        return convertRectangularToPolar(new double[]{x, y});
    }

    /**
     * given an array of polar coordinates (r, theta), returns array of equivalent rectangular coordinates (x,y)
     * @param polarCoords array of length 2 containing polar (r,theta) coordinates
     * @return array of length 2 containing equivalent rectangular coordinates (or null values if passed array is too big / too small)
     */
    public static double[] convertPolarToRectangular(double[] polarCoords) {
        //error prevention
        if (polarCoords.length != 2) return new double[2];
        else {
            //calculations
            double radius   = polarCoords[0];
            double theta    = polarCoords[1];
            double x        = Math.cos(theta) * radius;
            double y        = Math.sin(theta) * radius;

            return new double[]{x,y};
        }
    }
    /**
     * given polar coordinates (r, theta), returns array of equivalent rectangular coordinates (x,y)
     * @param radius radius, distance
     * @param theta  theta, angle
     * @return array of length 2 containing equivalent rectangular coordinates
     */
    public static double[] convertPolarToRectangular(double radius, double theta) {
        return convertPolarToRectangular(new double[]{radius, theta});
    }
}
