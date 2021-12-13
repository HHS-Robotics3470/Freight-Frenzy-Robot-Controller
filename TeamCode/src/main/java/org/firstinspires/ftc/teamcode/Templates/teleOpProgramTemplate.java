package org.firstinspires.ftc.teamcode.Templates;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;

@TeleOp(name="teleOpProgramTemplate")
@Disabled //this line disables the teleop from appearing on the driver station, remove it for your code
public class teleOpProgramTemplate extends LinearOpMode {
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

        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        runtime.reset();


        ////////////after driver presses play////////////
        //maybe some other set up stuff depending on how we want to do this

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
             /* CONTROLS
            joysticks: movement
                left joystick:
                right joystick:

            triggers and bumpers:
                left trigger:
                right trigger:
                left bumper:
                right bumper:

            ABXY:
                A:
                B:
                X:
                Y:

            D-Pad:
                Up:
                Down:
                Left:
                Right:

             */

            /////JOYSTICKS/////

            /////TRIGGERS AND BUMPERS/////

            /////ABXY/////

            /////D-Pad/////

        }

        ////////////after driver presses stop////////////
    }

    ////////////other methods and whatnot below here////////////


}
