package org.firstinspires.ftc.teamcode.Templates;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;

/**
 * Template for Autonomous routines
 * @author Anthony Rubick
 */
@Autonomous(name="Auto Name goes here", group="opmode group goes here" )
@Disabled //this line disables the autonomous from appearing on the driver station, remove it for your code
public class warehouseParkingAuto extends LinearOpMode{
    /*declare OpMode members, initialize some classes*/
    Hardware robot          = new Hardware();
    ElapsedTime runtime     = new ElapsedTime();

    //this is the control loop, basically the equivalent of a main function almost
    @Override
    public void runOpMode()
    {
        ////////////before driver presses play////////////
        //Variables
        double init_move = 0.1524;
        double angle_one = 4.71239;
        double angle_two = -0.785398;
        double second_move = 0.1524;
        double angle_three = -0.785398;
        double angle_four = 2.35619;
        double third_move = 1;



        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.initVuforiaAndTfod(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        ////////////after driver presses play////////////

        //autonomous routine goes here step by step

        //Orient robot so that turn does not hit the field wall
        robot.strafeToDistance(1, angle_one, init_move);
        robot.rotateByAngle(angle_two, 1);
        robot.strafeToDistance(1, angle_three, second_move);
        robot.turntableMotor.setPower(1);
        sleep(500);
        robot.turntableMotor.setPower(0);
        robot.strafeToDistance(1, angle_three, -second_move);
        robot.rotateByAngle(angle_four, 1);
        robot.strafeToDistance(1, angle_one, third_move);




        ////////////after driver presses stop////////////
    }

    ////////////other methods and whatnot below here////////////

}
