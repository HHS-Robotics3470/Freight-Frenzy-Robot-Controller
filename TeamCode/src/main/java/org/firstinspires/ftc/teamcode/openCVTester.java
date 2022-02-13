package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.hardware.android.GpioPin;
import org.firstinspires.ftc.teamcode.Hardware;

/**
 * Template for Autonomous routines
 * @author Anthony Rubick
 */
@Autonomous(name="Auto Name goes here", group="opmode group goes here" )
@Disabled //this line disables the autonomous from appearing on the driver station, remove it for your code
public class openCVTester  extends LinearOpMode{
    /*declare OpMode members, initialize some classes*/
    Hardware robot          = new Hardware();
    ElapsedTime runtime     = new ElapsedTime();

    //this is the control loop, basically the equivalent of a main function almost
    @Override
    public void runOpMode()
    {
        ////////////before driver presses play////////////
        //Variables


        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.initVuforiaAndTfod(hardwareMap); //uncomment if necessary


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        ////////////after driver presses play////////////

        //autonomous routine goes here step by step
        while (opModeIsActive()) {
            int level = robot.pipeline.getAnalysis();
            telemetry.addData("Position", robot.pipeline.getAnalysis());
            telemetry.addData("Center Average", robot.pipeline.getAnalysis());
            telemetry.addData("Right Average", robot.pipeline.getRightAvg());
            telemetry.update();
        }

        ////////////after driver presses stop////////////
    }

    ////////////other methods and whatnot below here////////////

}
