package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Template for Autonomous routines
 * @author Anthony Rubick
 */
@Autonomous(name="openCV tester", group="Testing" )
//@Disabled //this line disables the autonomous from appearing on the driver station, remove it for your code
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
        robot.initOpenCV(hardwareMap); //uncomment if necessary


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        ////////////after driver presses play////////////

        //autonomous routine goes here step by step
        while (opModeIsActive()) {
            int level = robot.opencvPipeline.getAnalysis();
            telemetry.addData("Position", robot.opencvPipeline.getAnalysis());
            telemetry.addData("Center Average", robot.opencvPipeline.getAnalysis());
            telemetry.addData("Right Average", robot.opencvPipeline.getRightAvg());
            telemetry.update();
        }

        ////////////after driver presses stop////////////
    }

    ////////////other methods and whatnot below here////////////

}
