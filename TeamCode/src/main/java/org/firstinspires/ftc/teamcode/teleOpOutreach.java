package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.math.BigDecimal;
import java.math.MathContext;

@TeleOp(name="TeleOP For Outreach Events", group="Outreach")
public class teleOpOutreach  extends LinearOpMode {
    /*declare OpMode members, initialize some classes*/
    HardwareOutreach robot          = new HardwareOutreach();
    ElapsedTime runtime     = new ElapsedTime();

    //other variables
    double currentTurretHeading = 0;
    double currentTurretPitch   = 0;

    //game control variables
    int ringsLeft = 5; //TODO update this
    int gameLengthSeconds = 5*60; //give them 5 minutes to play

    //variables for the elevation iteration algorithm
    double elevationStep = 1;
    MathContext elevationMC = new MathContext(16);
    BigDecimal elevationLastGuessDegrees = new BigDecimal("00.0000", elevationMC);
    double elevationGuessOffset = 1;

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
        while (opModeIsActive() && ringsLeft > 0 && (int)runtime.seconds() <= gameLengthSeconds) {
            //controls
            /* for controls:
            a,b,x,y pad will have turret controls
                a - reload and fire

            d-pad
                d-pad up    - elevate turret
                d-pad down  - lower turret
                d-pad left  - rotate turret CCW
                d-pad right - rotate turret CW

            thumbsticks will control movement
            */
            //////ABXY bindings/////
            //fire turret
            if (gamepad1.a) {

            }

            //////D-Pad bindings/////
            //elevate and lower
            if (gamepad1.dpad_up) {

            }
            else if (gamepad1.dpad_down) {

            }
            //rotate
            if (gamepad1.dpad_left) {

            }
            else if (gamepad1.dpad_right) {

            }

            //tell the user how many rings, and how much time they have remaining
            telemetry.addData("rings remaining: ", ringsLeft);
            telemetry.addData("time remaining: ", (double)gameLengthSeconds - runtime.seconds());
            telemetry.update();
        }

        ////////////after driver presses stop////////////
        telemetry.clearAll();
        telemetry.addLine("GAME OVER");
        telemetry.addLine("thanks for playing :D");
        telemetry.update();

        sleep(5000);
    }

    ////////////other methods and whatnot below here////////////
    /**
     * modification of the other rotateTurret method, this one disregards the robots heading
     * given the angle relative to the robot, move the turret to that angle
     * @param angle angle relative to robot
     * @return if the target angle in in a dead zone, it returns -1, otherwise it returns 0 and rotates the turret to the needed position
     */
    public int rotateTurretTo(double angle) {
        double targetPosition;

        //heading relative to field -> heading relative to the robot
        targetPosition = angle;
        //check if heading rel. to robot is in the deadzone, if so, return -1
        if (targetPosition > Math.toRadians(60) || targetPosition < -Math.toRadians(45)) {rotateTurretTo(0); return -1;}
        currentTurretHeading = Math.toDegrees(targetPosition);
        //convert the heading rel. to robot into the needed encoder counts
        targetPosition /= robot.CORE_HEX_RADIANS_PER_COUNTS;
        //make sure that the robot rotates the best direction to reach goal

        //rotate to that position and return 0
        robot.runMotorToPosition(robot.turretRotator, (int) targetPosition, 0.5);
        return 0;
    }


    /**
     * given the angle relative to the horizontal, move the turret to elevate to that pitch
     * @param angle desired angle of pitch
     * @return if the target angle in in a dead zone, it returns -1, otherwise it returns 0 and elevates the turret to the needed position
     */
    public double elevateTurretTo(double angle) {
        /* basic layout
        -return -1 if something went wrong, or the desired angle is impossible
        -elevate to position
        -return 0, to show that nothing went wrong

        more detail
        -get the needed pitch to the target from the AimManager
        -calculate what position the elevation servo needs to rotate to in order the acheive that pitch (trig)
        -if that position is a deadzone, would be blocked by other parts of the robot, or is otherwise not safe to move toward, return -1, terminating this process
        -if that position is able to be rotated to, then do that, and return 0
         */
        /*
        2 options, find a function to calculate the required angle, or use iteration https://www.desmos.com/calculator/omowmwxgj2
         */
        //iteration
        //reset the global values that control the iteration
        elevationStep = 1;
        elevationLastGuessDegrees = BigDecimal.ZERO;
        elevationGuessOffset = 1;
        //logic statement to make sure that the given target angle of the turret is possible, code in when range of motion is known (if (out of bounds) return -1;
        if (angle > Math.toRadians(40) || angle < 0) {elevateTurretTo(0); return -1;} // if the angle is greater than 40 degrees, or less than zero, stop, reset and return -1,

        //iterate and save
        BigDecimal targetPos = elevationCalculationIteration(angle); // in radians
        currentTurretPitch = Math.toDegrees(angle);
        //convert the target pos to a value in encoder counts
        targetPos = targetPos.divide(BigDecimal.valueOf(robot.GO_BILDA_RADIANS_PER_COUNTS), elevationMC); // in encoder counts

        robot.runMotorToPosition(robot.turretElevator, targetPos.intValue(), 0.5);
        return 0;
    }
    /**
     * this function uses iteration to find the optimal position to move the elevator motor to in order to elevate the turret to a given angle,
     * this function works best when working with degrees so it takes an input and gives an output in radians, while using degrees for internal calculations
     * @param targetAngle the angle, in radians
     * @return the angle to move the elevator to, in radians
     */
    public BigDecimal elevationCalculationIteration (double targetAngle) {
        while (elevationGuessOffsetCalculation(-targetAngle, Math.toRadians(-elevationLastGuessDegrees.doubleValue())) > 0) { //convert the target angle, and the guess, to radians in the call to the elevationGuessOffsetCalculation() method, this is because trig in java is done in radians
            elevationLastGuessDegrees = elevationLastGuessDegrees.add(BigDecimal.valueOf(elevationStep), elevationMC);
        }
        elevationLastGuessDegrees = elevationLastGuessDegrees.subtract(BigDecimal.valueOf(elevationStep), elevationMC);

        if (!(Math.abs(elevationGuessOffset) < 0.00001 || elevationStep < 0.0001)) { //skip iteration if the offset is close enough to zero, or it has iterated past 4 decimal places
            elevationStep /= 10;
            elevationCalculationIteration(targetAngle);
        }

        return (elevationLastGuessDegrees.multiply(BigDecimal.valueOf(Math.PI), elevationMC)).divide(BigDecimal.valueOf(180), elevationMC);
    }
    public double elevationGuessOffsetCalculation (double targetAngle, double currentGuess) {
        double x1 = Math.sin(targetAngle) * .02;
        double y1 = Math.cos(targetAngle) * .02;
        double x2 = Math.sin(currentGuess) * .02 + 0.0762;
        double y2 = Math.cos(currentGuess) * .02;
        double x3 = Math.cos(currentGuess) * .09525 + x2;
        double y3 = -Math.sin(currentGuess) * .09525 + y2;

        elevationGuessOffset = (y1 / x1) + ((x1-x3) / (y1-y3));
        return elevationGuessOffset;
    }

}

