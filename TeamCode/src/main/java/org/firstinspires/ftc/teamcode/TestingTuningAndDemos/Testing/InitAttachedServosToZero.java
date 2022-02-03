package org.firstinspires.ftc.teamcode.TestingTuningAndDemos.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;

import java.util.List;

/**
 * can cycle through and test every installed and configured servo
 * @author Anthony Rubick
 */
@TeleOp(name="T: Init servos to zero", group="Testing" )
//@Disabled //this line disables the teleop from appearing on the driver station, remove it when needed
public class InitAttachedServosToZero extends LinearOpMode {
    /*declare OpMode members, initialize some classes*/
    Hardware robot  = new Hardware();
    ElapsedTime runtime     = new ElapsedTime();

    //this is the control loop, basically the equivalent of a main function almost
    @Override
    public void runOpMode()
    {
        ////////////before driver presses play////////////
        //Variables
        double increment = 0.1;
        int listSize; //highest index filled in the list (# elements - 1)
        int currIndex = 0;

        //button locks
        boolean upCurr, downCurr, leftCurr, rightCurr, leftBCurr, rightBCurr;
        boolean upPrev = false, downPrev = false, leftPrev = false, rightPrev = false, leftBPrev = false, rightBPrev = false;


        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        // Define and initialize ALL installed servos.
        List<Servo> servoList = hardwareMap.getAll(Servo.class);
        Servo currentServo;


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        ////////////after driver presses play////////////
        //maybe some other set up stuff depending on how we want to do this
        listSize = servoList.size() - 1; //highest index filled in the list (# elements - 1)
        if (listSize < 0) // if no servos were loaded, notify the user and end the program
        {
            telemetry.addLine("ERROR:");
            telemetry.addLine(" - no servos connected/configured, or other issue loading servos");
            telemetry.update();
            sleep(5000);
            return;
        }
        currentServo = servoList.get(currIndex);

        // Set start positions for ALL installed servos
        for (Servo s : servoList)
        {
            s.setPosition(0.0);
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //code goes here

        }

        ////////////after driver presses stop////////////
    }

    ////////////other methods and whatnot below here////////////

}