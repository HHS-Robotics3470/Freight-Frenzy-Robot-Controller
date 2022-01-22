package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;


/**
 * Hovercraft control code
 *
 * @author Aaron Wong
 * @version 1/21/2022
 */
@TeleOp(name = "Hovercraft")
public class Hovercraft extends LinearOpMode
{
    private OpMode opmode;
    // declare hardware
    private DcMotor lift;
    private DcMotor prop;
    private Servo turn;
    private double servoMax = Math.PI; // 180 degrees
    @Override
    public void runOpMode() throws InterruptedException{
        //get hardware names linked with software names
        lift = hardwareMap.dcMotor.get("lift");
        prop = hardwareMap.dcMotor.get("prop");
        turn = hardwareMap.servo.get("servo");

        waitForStart();// wait for user to initialize

        //start the looping of the code
        while(opModeIsActive()){
            double xAxis = 0;
            double yAxis = 0;
            xAxis = opmode.gamepad1.left_stick_x;
            yAxis = opmode.gamepad1.left_stick_y;
            boolean liftStatus = false;

            //check when to turn on and off lift
            if (gamepad1.left_stick_button) {
                if (liftStatus){
                    lift.setPower(0);
                }
                else {
                    lift.setPower(1);
                }
            }
            // power of prop
            prop.setPower(propPower(xAxis,yAxis));



            //turn servo
            double radAngle = turnAngle(xAxis,yAxis);//Radians
            turn.setPosition(whichWayShortest(radAngle));


            sleep(100);
        }
    }

    /**
     * calculates the magnitude of the power of the prop fan
     */
    public double propPower(double x,double y){
        double answer = Math.sqrt(x * x + y * y);
        return answer;
    }

    /**
     * calculate how servo should turn
     */
    public double turnAngle( double x, double y){
        //change to atan2 if/when you use a 360 servo
        double angle =  Math.atan(y/x);
        return angle;
    }

    /**
     * convert the passed angle to an angle for the servo to rotate to
     * @param angle angle to rotate to, between pi/2 and -pi/2
     * @return
     */
    public double whichWayShortest(double angle){
        //servo: [0,1] representing [0,180]
        //angle: [-90,90] -> [0,180]
        angle += Math.PI/2.0;
        //ensure angle is between 0 to 180 (this limit is caused by using atan rather than atan2
        angle = Math.abs(angle%servoMax);

        double position = angle/Math.PI; //convert angle to [0,1]
        return position;
        /*
        double rotate = 0;
        double wantPosition = -4*Math.PI* angle + 2*Math.PI;
        double currentPosition = turn.getPosition();
        if (Math.abs(currentPosition - wantPosition)<= Math.PI){
            rotate  = -Math.abs(currentPosition - wantPosition);
        }
        else{
            rotate = 360 - Math.abs(currentPosition - wantPosition);
        }
        return rotate;
        */
    }
}
