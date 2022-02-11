package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.CascadeOutputSystem;

/**
 * @author Anthony Rubick
 */
@TeleOp(name="mecanum TeleOp no cascade", group="Competition")
public class   MecanumTeleNoCascade  extends LinearOpMode {
    /*declare OpMode members, initialize some classes*/
    //an enum to represent states of input and output grabbers
    public enum IOState {
        /**
         * for input, this is full open, used when collecting an element
         * for output, this is drop-off position, meaning it's depositing a game element
         *
         */
        OPEN,
        CLOSED,
        /**
         * basically means that a process is using the input/output,
         *
         * for input, this is half open, used when dropping an item off into the output
         * for output, this is receiving position, used when receiving an element from input
         */
        RESTRICTED
    }
    //an enum to represent the states of process attached to the 'a' button
    public enum AProcess {
        NOT_STARTED,
        IN_PROGRESS,
        ENDED
    }
    //an enum to represent the states of process attached to the 'b' button
    public enum BProcess {
        NOT_STARTED,
        STAGE_ONE,
        STAGE_TWO,
        STAGE_THREE,
        ENDING
    }


    Hardware robot          = new Hardware();
    //ElapsedTime runtime     = new ElapsedTime();
    ElapsedTime bProcessTimer = new ElapsedTime();
    ElapsedTime inputTimer = new ElapsedTime();

    //this is the control loop, basically the equivalent of a main function almost
    @Override
    public void runOpMode()
    {
        ////////////before driver presses play////////////
        //Variables
        //state enums
        IOState inputState = IOState.OPEN;
        IOState outputState = IOState.CLOSED;
        AProcess aState = AProcess.NOT_STARTED; //state of the process attached to the 'a' button
        BProcess bState = BProcess.NOT_STARTED; //state of the process attached to the 'b' button
        //button locks
        boolean aC, bC, yC, xC, upC, downC; //is currently pressed
        boolean aP = false, bP = false, yP = false, xP = false, upP = false, downP = false; //was previously pressed
        //other
        //CascadeOutputSystem.OutputArmPosition tempOutArmPos;
        boolean outputArmRestricted = false;
        boolean outputArmAtTarget = false; //output of robot.cascadeOutputSystem.stepArmTowardTarget()
        //int cascadeCount = 0;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        //robot.initVuforiaAndTfod(hardwareMap); //uncomment if necessary
        //tempOutArmPos = CascadeOutputSystem.OutputArmPosition.RETRACTED; //initialize this as the retracted position
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //runtime.reset();
        bProcessTimer.reset();
        inputTimer.reset();


        ////////////after driver presses play////////////
        //maybe some other set up stuff depending on how we want to do this
        //raise arm
        robot.intakeSystem.intakeGrabberServo.setPosition(robot.intakeSystem.GRABBER_CLOSED);
        robot.intakeSystem.intakeArmServo.setPosition(robot.intakeSystem.ARM_RAISED);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            /* CONTROLS
            joysticks: movement
                left joystick: strafe
                right joystick: rotation

            triggers and bumpers: unassigned

            ABXY: special functions
                A: grab item w/ input servo, press and hold to lower intake and open grip, release to close grip and raise input
                B: flip intake and drop off in output basket (retract if necessary)
                X: drop item off (open/close output servo)
                Y: if output flipper is extended, retract, otherwise extend to MIDDLE

            D-Pad:
                Up: if output flipper is extended MIDDLE, go to extended up; if output flipper is extended down, go to extended MIDDLE; if output flipper is extended up, extend down
                Down: if output flipper is extended MIDDLE, go to extended down; if output flipper is extended up, go to extended MIDDLE; if output flipper is extended down, extend up
                Left: extend cascade kit
                Right: retract cascade kit

             */
            //update button locks
            aC = gamepad1.a;
            bC = gamepad1.b;
            xC = gamepad1.x;
            yC = gamepad1.y;
            upC = gamepad1.dpad_up;
            downC = gamepad1.dpad_down;
            //joystick values
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double r = gamepad1.right_stick_x/2.0;
            //these hardware calls take 27ms total (3ms each)

            /////JOYSTICKS/////
            //handles strafing and turning in one optimized step.
            //because the joysticks give rectangular coordinates, there's not much point converting to polar than back to rectangular
            double denom = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(r) , 1.0);
            robot.driveTrain.setPower(
                    (y-x-r)/denom,
                    (y+x+r)/denom,
                    (y-x+r)/denom,
                    (y+x-r)/denom
            ); //+12ms


            /////TRIGGERS AND BUMPERS/////

            /////ABXY/////
            //A input
            switch (aState) {
                case NOT_STARTED:
                    if (aC&&!aP) { //on button press, lower arm
                        robot.intakeSystem.intakeGrabberServo.setPosition(robot.intakeSystem.GRABBER_FULL_OPEN);
                        inputState = IOState.OPEN;
                        //skip to end
                        robot.intakeSystem.intakeArmServo.setPosition(robot.intakeSystem.ARM_DOWN);
                        aState = AProcess.ENDED;
                    }
                    else if (aP&&!aC) { //on button release, raise arm again
                        robot.intakeSystem.intakeGrabberServo.setPosition(robot.intakeSystem.GRABBER_CLOSED);
                        inputState = IOState.CLOSED;
                        //set up for rest of the states
                        aState = AProcess.IN_PROGRESS;
                        inputTimer.reset();
                    }
                    break;
                case IN_PROGRESS:
                    //wait 400ms to give flipper time to raise / lower
                    if (inputTimer.seconds() >= .400) {
                        robot.intakeSystem.intakeArmServo.setPosition(robot.intakeSystem.ARM_RAISED);
                        aState = AProcess.ENDED;
                    }
                    break;
                case ENDED:
                default:
                    aState = AProcess.NOT_STARTED;
                    break;
            }
            //B transfer input to output
            //tempOutArmPos saves the position of the flipper so we can return to it later
            switch (bState) {
                case NOT_STARTED:
                    if (bC&&!bP) {
                        //save some info about the robots state so we can return to it at the end
                        //tempOutArmPos = robot.cascadeOutputSystem.getOutputArmPosition();

                        if (inputState != IOState.CLOSED) {
                            robot.intakeSystem.intakeGrabberServo.setPosition(robot.intakeSystem.GRABBER_CLOSED);//close input
                        }
                        robot.cascadeOutputSystem.outputGrabberServo.setPosition(robot.cascadeOutputSystem.GRABBER_RECEIVE);//open output

                        //retract
                        //robot.cascadeOutputSystem.outputArmServo.setPosition(robot.cascadeOutputSystem.ARM_RETRACTED);//fold back
                        robot.cascadeOutputSystem.setOutputArmPosition(CascadeOutputSystem.OutputArmPosition.RETRACTED);

                        //restrict relevant systems
                        inputState = IOState.RESTRICTED;
                        outputState = IOState.RESTRICTED;
                        outputArmRestricted = true;
                        bState = BProcess.STAGE_ONE; //go to next stage
                        bProcessTimer.reset();
                    }
                    break;
                case STAGE_ONE:
                    //wait for retraction
                    if (outputArmAtTarget) {
                        //raise front input flipper to drop off element
                        robot.intakeSystem.intakeArmServo.setPosition(robot.intakeSystem.ARM_UP);
                        bState = BProcess.STAGE_TWO; //go to next stage
                        bProcessTimer.reset();
                    }
                    break;
                case STAGE_TWO:
                    //wait 700ms for input flipper to raise fully
                    if (bProcessTimer.seconds() >= .700) {
                        robot.intakeSystem.intakeGrabberServo.setPosition(robot.intakeSystem.GRABBER_PARTIAL_OPEN); //open input

                        bState = BProcess.STAGE_THREE; // go to next stage
                        bProcessTimer.reset();
                    }
                    break;
                case STAGE_THREE:
                    //wait 300ms to give time for the game element to transfer before re-extending cascade
                    if (bProcessTimer.seconds() >= .300) {
                        robot.cascadeOutputSystem.outputGrabberServo.setPosition(robot.cascadeOutputSystem.GRABBER_CLOSED); //close output
                        robot.intakeSystem.intakeArmServo.setPosition(robot.intakeSystem.ARM_RAISED);
                        robot.intakeSystem.intakeGrabberServo.setPosition(robot.intakeSystem.GRABBER_FULL_OPEN);

                        bState = BProcess.ENDING; //go to next stage
                        bProcessTimer.reset();
                    }
                    break;
                case ENDING:
                    //wait for re-extension, then ...
                    if (outputArmAtTarget) {
                        //update robot state
                        outputArmRestricted = false;
                        outputState = IOState.CLOSED;
                        inputState = IOState.OPEN;
                        bState = BProcess.NOT_STARTED; //back to start
                        bProcessTimer.reset();
                    }
                    break;
                default:
                    bState = BProcess.NOT_STARTED; //back to start
                    break;
            }
            //X output
            if (xC&&!xP) {
                switch (outputState) {
                    case OPEN:
                        robot.cascadeOutputSystem.outputGrabberServo.setPosition(robot.cascadeOutputSystem.GRABBER_CLOSED); //close
                        outputState = IOState.CLOSED;
                        break;
                    case CLOSED:
                        if (robot.cascadeOutputSystem.getOutputArmPosition().equals(CascadeOutputSystem.OutputArmPosition.UP)) {
                            robot.cascadeOutputSystem.outputGrabberServo.setPosition(robot.cascadeOutputSystem.GRABBER_RECEIVE);
                            sleep(200);
                        }
                        robot.cascadeOutputSystem.outputGrabberServo.setPosition(robot.cascadeOutputSystem.GRABBER_DROP);
                        outputState = IOState.OPEN;
                        break;
                    case RESTRICTED:
                    default:
                        break;
                }
            }
            //Y implemented later, bundled w/ UP and DOWN

            /////D-Pad/////
            //UP DOWN Y
            if (!outputArmRestricted) {
                switch (robot.cascadeOutputSystem.getOutputArmPosition()) {
                    case RETRACTED:
                        if (yC && !yP) {
                            //robot.cascadeOutputSystem.outputArmServo.setPosition(robot.cascadeOutputSystem.ARM_EXTENDED_MIDDLE); //extend MIDDLE
                            robot.cascadeOutputSystem.setOutputArmPosition(CascadeOutputSystem.OutputArmPosition.UP); //extend to the up position
                        }
                        break;
                    case UP:
                        if (upC && !upP) {
                            //robot.cascadeOutputSystem.outputArmServo.setPosition(robot.cascadeOutputSystem.ARM_EXTENDED_DOWN); //down
                            robot.cascadeOutputSystem.setOutputArmPosition(CascadeOutputSystem.OutputArmPosition.DOWN);
                        } else if (downC && !downP) {
                            //robot.cascadeOutputSystem.outputArmServo.setPosition(robot.cascadeOutputSystem.ARM_EXTENDED_MIDDLE); //MIDDLE
                            robot.cascadeOutputSystem.setOutputArmPosition(CascadeOutputSystem.OutputArmPosition.MIDDLE);
                        } else if (yC && !yP) {
                            //robot.cascadeOutputSystem.outputArmServo.setPosition(robot.cascadeOutputSystem.ARM_RETRACTED); //retract
                            robot.cascadeOutputSystem.setOutputArmPosition(CascadeOutputSystem.OutputArmPosition.RETRACTED);
                        }
                        break;
                    case MIDDLE:
                        if (upC && !upP) {
                            //robot.cascadeOutputSystem.outputArmServo.setPosition(robot.cascadeOutputSystem.ARM_EXTENDED_UP); //up
                            robot.cascadeOutputSystem.setOutputArmPosition(CascadeOutputSystem.OutputArmPosition.UP);
                        } else if (downC && !downP) {
                            //robot.cascadeOutputSystem.outputArmServo.setPosition(robot.cascadeOutputSystem.ARM_EXTENDED_DOWN); //down
                            robot.cascadeOutputSystem.setOutputArmPosition(CascadeOutputSystem.OutputArmPosition.DOWN);
                        } else if (yC && !yP) {
                            //robot.cascadeOutputSystem.outputArmServo.setPosition(robot.cascadeOutputSystem.ARM_RETRACTED); //retract
                            robot.cascadeOutputSystem.setOutputArmPosition(CascadeOutputSystem.OutputArmPosition.RETRACTED);
                        }
                        break;
                    case DOWN:
                        if (upC && !upP) {
                            //robot.cascadeOutputSystem.outputArmServo.setPosition(robot.cascadeOutputSystem.ARM_EXTENDED_MIDDLE); //MIDDLE
                            robot.cascadeOutputSystem.setOutputArmPosition(CascadeOutputSystem.OutputArmPosition.MIDDLE);
                        } else if (downC && !downP) {
                            //robot.cascadeOutputSystem.outputArmServo.setPosition(robot.cascadeOutputSystem.ARM_EXTENDED_UP); //up
                            robot.cascadeOutputSystem.setOutputArmPosition(CascadeOutputSystem.OutputArmPosition.UP);
                        } else if (yC && !yP) {
                            //robot.cascadeOutputSystem.outputArmServo.setPosition(robot.cascadeOutputSystem.ARM_RETRACTED); //retract
                            robot.cascadeOutputSystem.setOutputArmPosition(CascadeOutputSystem.OutputArmPosition.RETRACTED);
                        }
                        break;
                    default:
                        //should never be reached, output flipper state should never be null
                        //robot.cascadeOutputSystem.outputArmServo.setPosition(robot.cascadeOutputSystem.ARM_RETRACTED); //retract
                        robot.cascadeOutputSystem.setOutputArmPosition(CascadeOutputSystem.OutputArmPosition.RETRACTED);
                        break;
                }
            }
            //step output arm toward target
            outputArmAtTarget = robot.cascadeOutputSystem.stepArmTowardTarget();

            //LEFT
            if (gamepad1.dpad_left) {
                robot.turntableMotor.setPower(Hardware.TURNTABLE_SPEED);
            }
            //RIGHT
            else if (gamepad1.dpad_right) {
                robot.turntableMotor.setPower(-Hardware.TURNTABLE_SPEED);
            }
            else robot.turntableMotor.setPower(0);

            //telemetry on robot state
            runTelemetry(robot.cascadeOutputSystem.getOutputArmPosition(), outputState, inputState, aState, bState);

            //update button locks
            aP=aC;
            bP=bC;
            xP=xC;
            yP=yC;
            upP = upC;
            downP = downC;
        }
        ////////////after driver presses stop////////////

    }

    ////////////other methods and whatnot below here////////////
    public void runTelemetry(CascadeOutputSystem.OutputArmPosition outputArmState, IOState outputState, IOState inputState, AProcess aState, BProcess bState) {
        telemetry.addLine("---====STATES====---");
        telemetry.addData("Output Arm: ", outputArmState.toString());
        telemetry.addData("Output: ", outputState.toString());
        telemetry.addData("Input: ", inputState.toString());
        telemetry.addData("\nA-button process: ", aState.toString());
        telemetry.addData("B-button process: ", bState.toString());
        telemetry.update();
    }

}
