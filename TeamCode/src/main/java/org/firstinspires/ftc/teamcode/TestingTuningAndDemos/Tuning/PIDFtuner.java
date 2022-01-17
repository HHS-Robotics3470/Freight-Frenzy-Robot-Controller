package org.firstinspires.ftc.teamcode.TestingTuningAndDemos.Tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;

import java.util.List;


/*
basic idea, let the user cycle through every motor on the robot

when they press 'a', call a function that goes through the process of calculating PIDF values
 */

/**
 * can cycle through, and calculate PIDF coefficients appropriate for any installed and initialized motor motor
 *
 * untested
 * 
 * @author Anthony Rubick
 */
@TeleOp(name="T: PIDF tuner", group = "Tuning")
//@Disabled
//this line disables the teleop from appearing on the driver station, remove it for your code
public class PIDFtuner extends LinearOpMode {
    /*declare OpMode members, initialize some classes*/
    public enum ProcessState {
        /**
         * user is choosing the motor to test
         */
        CHOOSING,
        SHOWING_WARNINGS,
        /**
         * determining max velocity
         */
        MAX_VELOCITY,
        /**
         * using max velocity, calculate PIDF values
         */
        CALCULATING,
        /**
         * using new PIDF values, move to a position
         */
        TEST_ONE,
        TEST_TWO,
        /**
         * ask user to verify that it worked
         */
        VERIFYING,
        /**
         * display the PIDF values, maybe save them to a file too
         */
        SAVING,
        /**
         * finished
         */
        DONE,
    }

    Hardware robot          = new Hardware();
    ElapsedTime runtime     = new ElapsedTime();
    ElapsedTime buttonTimer = new ElapsedTime();

    /** as of 12.12.2021, power of 0.006 makes it barely move *on concrete* */

    //this is the control loop, basically the equivalent of a main function almost
    @Override
    public void runOpMode()
    {
        ////////////before driver presses play////////////
        //Variables
        List<DcMotorEx> motorList = hardwareMap.getAll(DcMotorEx.class);
        DcMotorEx currentMotor = motorList.get(0);
        int listSize = motorList.size()-1, currIndex=0;
        ProcessState state = ProcessState.CHOOSING;

        //variables read from hardware components
        //read every loop
        int encoderCounts;
        double velocity, maxVelocity=0; //current and maximum motor velocity (encoder ticks per second)
        //read once when motor changes
        String motorName = currentMotor.getDeviceName();
        int portNum = currentMotor.getPortNumber();
        PIDFCoefficients velPidfCoefficients = currentMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        String controlAlgorithm = velPidfCoefficients.toString().substring(velPidfCoefficients.toString().lastIndexOf("alg="));//TODO: find out how to extract just the control algorithm name from the string
        double posP = currentMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).p; //p coefficient for position PIDF

        //double newP; //p coefficient for position PIDF
        PIDFCoefficients newVelPidfCoefficients = velPidfCoefficients;


        //button locks
        boolean uc, dc,//d-pad
                lbc, rbc, //left/right bumpers
                ac,bc,xc,yc; //abxy
        boolean lbp=false, rbp=false,
                ap=false,bp=false,xp=false,yp=false;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap); //runs this to get motor positions
        //configure ALL installed motors
        for (DcMotorEx m : motorList)
        {
            m.setPower(0);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        buttonTimer.reset();
        runtime.reset();


        ////////////after driver presses play////////////
        //maybe some other set up stuff depending on how we want to do this

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            encoderCounts = currentMotor.getCurrentPosition();
            velocity = currentMotor.getVelocity();
            uc  = gamepad1.dpad_up;
            dc  = gamepad1.dpad_down;
            lbc = gamepad1.left_bumper;
            rbc = gamepad1.right_bumper;
            ac  = gamepad1.a;
            bc  = gamepad1.b;
            xc  = gamepad1.x;
            yc  = gamepad1.y;
            boolean lb=lbc&&!lbp;
            boolean rb=rbc&&!rbp;
            boolean a = ac&!ap;
            boolean b=bc&&!bp;
            boolean x=xc&&!xp;
            boolean y=yc&&!yp;


            /* CONTROLS
            joysticks: movement
                left joystick:
                right joystick:

            triggers and bumpers:
                left trigger:
                right trigger:
                left bumper: cycle motors in CHOOSING state
                right bumper: cycle motors in CHOOSING state

            ABXY:
                A: starts calculating new PIDF coefficients
                B: aborts calculation
                X: skips warnings in SHOWING_WARNINGS state, also acts as the "no" input in the VERIFYING state
                Y: acts as the "yes" input in the VERIFYING state

            D-Pad:
                Up: move currentMotor forward in CHOOSING state
                Down: move currentMotor backward in CHOOSING state
                Left:
                Right:
             */

            switch (state) {
                case CHOOSING:
                    if (lb) {
                        // set currentMotor to the previous index in motor list
                        currIndex--;
                        if (currIndex <= 0) currIndex = listSize; //lower bound
                        currentMotor = motorList.get(currIndex);
                        motorName = currentMotor.getDeviceName();
                        portNum = currentMotor.getPortNumber();
                        velPidfCoefficients = currentMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                        controlAlgorithm =  velPidfCoefficients.toString().substring(velPidfCoefficients.toString().lastIndexOf("alg="));
                        posP = currentMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).p; //p coefficient for position PIDF
                    } else if (rb) {
                        // set currentMotor to the next index in motor list
                        currIndex++;
                        if (currIndex > listSize) currIndex = 0;
                        currentMotor = motorList.get(currIndex);
                        motorName = currentMotor.getDeviceName();
                        portNum = currentMotor.getPortNumber();
                        velPidfCoefficients = currentMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                        controlAlgorithm =  velPidfCoefficients.toString().substring(velPidfCoefficients.toString().lastIndexOf("alg="));
                        posP = currentMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).p; //p coefficient for position PIDF
                    }

                    if (uc) {
                        currentMotor.setPower(0.5);
                    } else if (dc) {
                        currentMotor.setPower(-0.5);
                    } else {
                        currentMotor.setPower(0);
                    }

                    if (a) {
                        state=ProcessState.SHOWING_WARNINGS;
                        buttonTimer.reset();
                        break;
                    }

                    generalTelemetry(motorName, portNum, encoderCounts, velocity, maxVelocity, controlAlgorithm, velPidfCoefficients);
                    break;
                case SHOWING_WARNINGS:
                    //if time has run out, or button x pressed, continue
                    if (buttonTimer.seconds() >= 5.0 || x) {
                        currentMotor.setPower(1);

                        buttonTimer.reset();
                        state = ProcessState.MAX_VELOCITY;
                        break;
                    }

                    //show warnings
                    warningTelemetry(currentMotor.getDeviceName(),5.0-buttonTimer.seconds());
                    break;
                case MAX_VELOCITY:
                    if (velocity > maxVelocity) {
                        maxVelocity = velocity;
                    }
                    //run test for 3 seconds
                    if (buttonTimer.seconds() >= 3.0) {
                        currentMotor.setPower(0);
                        state = ProcessState.CALCULATING;
                        break;
                    }

                    generalTelemetry(motorName, portNum, encoderCounts, velocity, maxVelocity, controlAlgorithm, velPidfCoefficients);
                    break;
                case CALCULATING:
                    //calculate new pidf coefficients
                    double p,i,f;
                    f = 32767.0 / maxVelocity;
                    p = 0.1*f;
                    i = 0.1*p;
                    newVelPidfCoefficients = new PIDFCoefficients(p,i,0,f);

                    //assign motor those coefficients
                    try {
                        currentMotor.setVelocityPIDFCoefficients(p, i, 0, f);
                        currentMotor.setPositionPIDFCoefficients(posP);
                    } catch (UnsupportedOperationException e) {
                        //change algorithm and try again
                        currentMotor.getMotorType().getHubVelocityParams().algorithm = MotorControlAlgorithm.PIDF;
                        currentMotor.getMotorType().getHubPositionParams().algorithm = MotorControlAlgorithm.PIDF;
                        break;
                    }

                    generalTelemetry(motorName, portNum, encoderCounts, velocity, maxVelocity, controlAlgorithm, newVelPidfCoefficients);
                    state = ProcessState.TEST_ONE;
                    break;
                case TEST_ONE:
                    //rotate back at 1/3 power
                    //currentMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    currentMotor.setTargetPosition( 1440 );
                    currentMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    currentMotor.setVelocity(maxVelocity/3);

                    generalTelemetry(motorName, portNum, encoderCounts, velocity, maxVelocity, controlAlgorithm, newVelPidfCoefficients);
                    state=ProcessState.TEST_TWO;
                    break;
                case TEST_TWO:
                    if (!currentMotor.isBusy()) {
                        //rotate forward at full power
                        currentMotor.setVelocity(0);
                        //currentMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        currentMotor.setTargetPosition( 4320 );
                        currentMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        currentMotor.setVelocity(maxVelocity);

                        buttonTimer.reset();
                        state=ProcessState.VERIFYING;
                        break;
                    }

                    generalTelemetry(motorName, portNum, encoderCounts, velocity, maxVelocity, controlAlgorithm, newVelPidfCoefficients);
                    break;
                case VERIFYING:
                    telemetry.addLine("did PID work as expected?");
                    telemetry.addLine("\ty - yes\n\tx - no");
                    telemetry.addLine();
                    telemetry.addLine();
                    generalTelemetry(motorName, portNum, encoderCounts, velocity, maxVelocity, controlAlgorithm, newVelPidfCoefficients);

                    if (y) {
                        //continue
                        state = ProcessState.SAVING;
                    } else if (x) {
                        //stop
                        state = ProcessState.DONE;
                    }
                    break;
                case SAVING:
                    //maybe implement this later, would save PIDF data to a file, and/or permanently assign it to currentMotor
                case DONE:
                    state = ProcessState.CHOOSING;
                    break;
            }

            //rather than having this repeated in every state, do it once here, b aborts calculations and goes back to default state
            if (b) {
                state = ProcessState.CHOOSING;
                currentMotor.setPower(0);
                currentMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            //update button locks
            lbp=lbc;
            rbp=rbc;
            ap=ac;
            bp=bc;
            xp=xc;
            yp=yc;
        }

        ////////////after driver presses stop////////////
    }

    ////////////other methods and whatnot below here////////////
    private void generalTelemetry(String motorName, int portNum, int encoderCounts, double velocity, double maxVelocity, String controlAlgorithm, PIDFCoefficients origPidfCoefficients) {
        telemetry.addLine(String.format("Motor: %s", motorName));
        telemetry.addData("\tPort: ", portNum);
        telemetry.addData("\tEncoder Counts: ", encoderCounts);
        telemetry.addData("\tVelocity (ticks/sec): ", velocity);
        telemetry.addData("\t\tMax Velocity: ", maxVelocity);
        telemetry.addLine();
        telemetry.addData("\tControl algorithm: ", controlAlgorithm);
        telemetry.addLine("\tCoefficients");
        telemetry.addData("\t\tP: ", origPidfCoefficients.p);
        telemetry.addData("\t\tI: ", origPidfCoefficients.i);
        telemetry.addData("\t\tD: ", origPidfCoefficients.d);
        telemetry.addData("\t\tF: ", origPidfCoefficients.f);
        telemetry.update();
    }

    private void warningTelemetry(String motor, double secondsLeft) {
        telemetry.addData("Testing ", motor);
        telemetry.addLine("Make sure that:");
        telemetry.addLine("\n\t-the battery if fully charged");
        telemetry.addLine("\n\t-motor can spin freely (but has a load similar to what it will face in the field)");
        telemetry.addLine("\n\t-motor has an encoder attached");
        telemetry.addLine(String.format("x - skip\nb - back\n\n\tTime Remaining - %s s", secondsLeft));
        telemetry.update();
    }

}
