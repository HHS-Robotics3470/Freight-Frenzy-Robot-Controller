package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="mecanum TeleOp", group="Competition")
public class MecanumTeleOp extends LinearOpMode {
    /*declare OpMode members, initialize some classes*/
    Hardware robot          = new Hardware();
    ElapsedTime runtime     = new ElapsedTime();

    //this is the control loop, basically the equivalent of a main function almost
    @Override
    public void runOpMode()
    {
        ////////////before driver presses play////////////
        //Variables

        //button locks
        boolean aC, bC, yC, xC, upC, downC; //currently pressed
        boolean aP = false, bP = false, yP = false, xP = false, upP = false, downP = false; //previously pressed


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
                left joystick: strafe
                right joystick: rotation

            triggers and bumpers: unassigned

            ABYX: special functions
                A: grap item w/ input servo (open/close input servo)
                B: flip intake and drop off in output basket (retract if neccessary)
                Y: if output flipper is extended, retract, otherwise extend to flat
                X: drop item off (open/close output servo)

            D-Pad:
                Up: if output flipper is extended flat, go to extended up; if output flipper is extended down, go to extended flat; if output flipper is retracted, extend flat
                Down: if output flipper is extended flat, go to extended down; if output flipper is extended up, go to extended flat; if output flipper is retracted, extend flat
                Left: extend cascade kit
                Right: retract cascade kit

             */

            /////JOYSTICKS/////

            /////TRIGGERS AND BUMPERS/////

            /////ABXY/////
            //A


            /////D-Pad/////

        }

        ////////////after driver presses stop////////////
    }

    ////////////other methods and whatnot below here////////////


}
