package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * @author Anthony Rubick
 */
@TeleOp(name="mecanum TeleOp no cascade", group="Competition")
public class   MecanumTeleNoCascade  extends LinearOpMode {
    /*declare OpMode members, initialize some classes*/
    //an enum to represent output flipper states
    public enum OutputArmState {
        RETRACTED,
        UP,
        MIDDLE,
        DOWN,
        RESTRICTED
    }
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
    //an enum to represent the states of the processes attached to the up, down, and y buttons
    public enum OutputArmServoMovementProcess {
        AT_TARGET_POSITION,
        MOVING_TO_RETRACTED,
        MOVING_TO_UP,
        MOVING_TO_MIDDLE,
        MOVING_TO_DOWN,
        ENDING
    }


    Hardware robot          = new Hardware();
    ElapsedTime runtime     = new ElapsedTime();
    ElapsedTime bProcessTimer = new ElapsedTime();
    ElapsedTime inputTimer = new ElapsedTime();
    ElapsedTime servoTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    //this is the control loop, basically the equivalent of a main function almost
    @Override
    public void runOpMode()
    {
        ////////////before driver presses play////////////
        //Variables
        //state enums
        OutputArmState outputArmState = OutputArmState.RETRACTED;
        IOState inputState = IOState.OPEN;
        IOState outputState = IOState.CLOSED;
        AProcess aState = AProcess.NOT_STARTED; //state of the process attached to the 'a' button
        BProcess bState = BProcess.NOT_STARTED; //state of the process attached to the 'b' button
        OutputArmServoMovementProcess outputArmServoMovementProcess = OutputArmServoMovementProcess.AT_TARGET_POSITION;
        //button locks
        boolean aC, bC, yC, xC, upC, downC; //is currently pressed
        boolean aP = false, bP = false, yP = false, xP = false, upP = false, downP = false; //was previously pressed
        //other
        double tempOutArmPos;
        //int cascadeCount = 0;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        //robot.initVuforiaAndTfod(hardwareMap); //uncomment if necessary
        tempOutArmPos = robot.cascadeOutputSystem.ARM_RETRACTED; //initialize this as the retracted position
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        bProcessTimer.reset();
        inputTimer.reset();
        servoTimer.reset();


        ////////////after driver presses play////////////
        //maybe some other set up stuff depending on how we want to do this

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            /* CONTROLS
            joysticks: movement
                left joystick: strafe
                right joystick: rotation

            triggers and bumpers: unassigned

            ABXY: special functions
                A: grab item w/ input servo (open/close input servo)
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
                    if (aC&&!aP) {
                        switch (inputState) {
                            case OPEN:
                                robot.intakeSystem.intakeGrabberServo.setPosition(robot.intakeSystem.GRABBER_CLOSED);
                                inputState = IOState.CLOSED;
                                //set up for rest of the states
                                aState = AProcess.IN_PROGRESS;
                                inputTimer.reset();
                                break;
                            case CLOSED:
                                robot.intakeSystem.intakeGrabberServo.setPosition(robot.intakeSystem.GRABBER_FULL_OPEN);
                                inputState = IOState.OPEN;
                                //skip to end
                                robot.intakeSystem.intakeArmServo.setPosition(robot.intakeSystem.ARM_DOWN);
                                aState = AProcess.ENDED;
                                break;
                            default:
                                break;
                        }

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
                        tempOutArmPos = robot.cascadeOutputSystem.outputArmServo.getPosition();

                        if (inputState != IOState.CLOSED) {
                            robot.intakeSystem.intakeGrabberServo.setPosition(robot.intakeSystem.GRABBER_CLOSED);//close input
                        }
                        robot.cascadeOutputSystem.outputGrabberServo.setPosition(robot.cascadeOutputSystem.GRABBER_RECEIVE);//open output
                        //robot.cascadeOutputSystem.outputArmServo.setPosition(robot.cascadeOutputSystem.ARM_RETRACTED);//fold back
                        outputArmServoMovementProcess = OutputArmServoMovementProcess.MOVING_TO_RETRACTED;

                        //restrict relevant systems
                        inputState = IOState.RESTRICTED;
                        outputState = IOState.RESTRICTED;
                        outputArmState = OutputArmState.RESTRICTED;
                        bState = BProcess.STAGE_ONE; //go to next stage
                        bProcessTimer.reset();
                    }
                    break;
                case STAGE_ONE:
                    //wait for retraction
                    if (outputArmServoMovementProcess.equals(OutputArmServoMovementProcess.AT_TARGET_POSITION)) {
                        //raise front input flipper to drop off element
                        robot.intakeSystem.intakeArmServo.setPosition(robot.intakeSystem.ARM_UP);
                        bState = BProcess.STAGE_TWO; //go to next stage
                        bProcessTimer.reset();
                    }
                    break;
                case STAGE_TWO:
                    //wait 700ms for input flipper to raise fully
                    if (bProcessTimer.seconds() >= .700) {
                        robot.cascadeOutputSystem.outputGrabberServo.setPosition(robot.cascadeOutputSystem.GRABBER_CLOSED); //close output
                        robot.intakeSystem.intakeGrabberServo.setPosition(robot.intakeSystem.GRABBER_PARTIAL_OPEN); //open input

                        bState = BProcess.STAGE_THREE; // go to next stage
                        bProcessTimer.reset();
                    }
                    break;
                case STAGE_THREE:
                    //wait 300ms to give time for the game element to transfer before re-extending cascade
                    if (bProcessTimer.seconds() >= .300) {
                        robot.intakeSystem.intakeArmServo.setPosition(robot.intakeSystem.ARM_DOWN);
                        robot.intakeSystem.intakeGrabberServo.setPosition(robot.intakeSystem.GRABBER_FULL_OPEN); //fully open input

                        //re-extend output flipper
                        outputArmServoMovementProcess = OutputArmServoMovementProcess.MOVING_TO_MIDDLE;
                        bState = BProcess.ENDING; //go to next stage
                        bProcessTimer.reset();
                    }
                    break;
                case ENDING:
                    //wait for re-extension, then ...
                    if (outputArmServoMovementProcess == OutputArmServoMovementProcess.AT_TARGET_POSITION) {
                        //update robot state
                        outputArmState = OutputArmState.MIDDLE;
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
                        if (outputArmState == OutputArmState.UP) {
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
            switch (outputArmState) {
                case RETRACTED:
                    if (yC&&!yP) {
                        //robot.cascadeOutputSystem.outputArmServo.setPosition(robot.cascadeOutputSystem.ARM_EXTENDED_MIDDLE); //extend MIDDLE
                        outputArmServoMovementProcess = OutputArmServoMovementProcess.MOVING_TO_MIDDLE;
                        outputArmState = OutputArmState.MIDDLE;
                    }
                    break;
                case UP:
                    if (upC&&!upP) {
                        //robot.cascadeOutputSystem.outputArmServo.setPosition(robot.cascadeOutputSystem.ARM_EXTENDED_DOWN); //down
                        outputArmServoMovementProcess = OutputArmServoMovementProcess.MOVING_TO_DOWN;
                        outputArmState = OutputArmState.DOWN;
                    } else if (downC&&!downP) {
                        //robot.cascadeOutputSystem.outputArmServo.setPosition(robot.cascadeOutputSystem.ARM_EXTENDED_MIDDLE); //MIDDLE
                        outputArmServoMovementProcess = OutputArmServoMovementProcess.MOVING_TO_MIDDLE;
                        outputArmState = OutputArmState.MIDDLE;
                    } else if (yC&&!yP) {
                        //robot.cascadeOutputSystem.outputArmServo.setPosition(robot.cascadeOutputSystem.ARM_RETRACTED); //retract
                        outputArmServoMovementProcess = OutputArmServoMovementProcess.MOVING_TO_RETRACTED;
                        outputArmState = OutputArmState.RETRACTED;
                    }
                    break;
                case MIDDLE:
                    if (upC&&!upP) {
                        //robot.cascadeOutputSystem.outputArmServo.setPosition(robot.cascadeOutputSystem.ARM_EXTENDED_UP); //up
                        outputArmServoMovementProcess = OutputArmServoMovementProcess.MOVING_TO_UP;
                        outputArmState = OutputArmState.UP;
                    } else if (downC&&!downP) {
                        //robot.cascadeOutputSystem.outputArmServo.setPosition(robot.cascadeOutputSystem.ARM_EXTENDED_DOWN); //down
                        outputArmServoMovementProcess = OutputArmServoMovementProcess.MOVING_TO_DOWN;
                        outputArmState = OutputArmState.DOWN;
                    } else if (yC&&!yP) {
                        //robot.cascadeOutputSystem.outputArmServo.setPosition(robot.cascadeOutputSystem.ARM_RETRACTED); //retract
                        outputArmServoMovementProcess = OutputArmServoMovementProcess.MOVING_TO_RETRACTED;
                        outputArmState = OutputArmState.RETRACTED;
                    }
                    break;
                case DOWN:
                    if (upC&&!upP) {
                        //robot.cascadeOutputSystem.outputArmServo.setPosition(robot.cascadeOutputSystem.ARM_EXTENDED_MIDDLE); //MIDDLE
                        outputArmServoMovementProcess = OutputArmServoMovementProcess.MOVING_TO_MIDDLE;
                        outputArmState = OutputArmState.MIDDLE;
                    } else if (downC&&!downP) {
                        //robot.cascadeOutputSystem.outputArmServo.setPosition(robot.cascadeOutputSystem.ARM_EXTENDED_UP); //up
                        outputArmServoMovementProcess = OutputArmServoMovementProcess.MOVING_TO_UP;
                        outputArmState = OutputArmState.UP;
                    } else if (yC&&!yP) {
                        //robot.cascadeOutputSystem.outputArmServo.setPosition(robot.cascadeOutputSystem.ARM_RETRACTED); //retract
                        outputArmServoMovementProcess = OutputArmServoMovementProcess.MOVING_TO_RETRACTED;
                        outputArmState = OutputArmState.RETRACTED;
                    }
                    break;
                case RESTRICTED:
                    break;
                default:
                    //should never be reached, output flipper state should never be null
                    //robot.cascadeOutputSystem.outputArmServo.setPosition(robot.cascadeOutputSystem.ARM_RETRACTED); //retract
                    outputArmServoMovementProcess = OutputArmServoMovementProcess.MOVING_TO_RETRACTED;
                    outputArmState = OutputArmState.RETRACTED;
                    break;
            }

            //LEFT
            if (gamepad1.dpad_left) {
                robot.turntableMotor.setPower(0.5);
            }
            //RIGHT
            else if (gamepad1.dpad_right) {
                robot.turntableMotor.setPower(-0.5);
            }
            else robot.turntableMotor.setPower(0);

            //TODO: maybe extract this to the cascadeOutputSystem.java class
            //switch that handles stepping of servo movement
            switch (outputArmServoMovementProcess) {
                case AT_TARGET_POSITION:
                    break;
                case MOVING_TO_RETRACTED:
                    if ((int)servoTimer.milliseconds() >= robot.SERVO_STEP_TIME) { //step time elapsed
                        //do next step (in if header)
                        if (robot.stepServoTowardTarget(robot.cascadeOutputSystem.outputArmServo,robot.cascadeOutputSystem.ARM_RETRACTED,robot.SERVO_STEP_SIZE)) {
                            //if at final position
                            outputArmServoMovementProcess = OutputArmServoMovementProcess.ENDING;
                        } else servoTimer.reset();
                        break;
                    }
                    break;
                case MOVING_TO_UP:
                    if ((int)servoTimer.milliseconds() >= robot.SERVO_STEP_TIME) { //step time elapsed
                        //do next step (in if header)
                        if (robot.stepServoTowardTarget(robot.cascadeOutputSystem.outputArmServo,robot.cascadeOutputSystem.ARM_EXTENDED_UP,robot.SERVO_STEP_SIZE)) {
                            //if at final position
                            outputArmServoMovementProcess = OutputArmServoMovementProcess.ENDING;
                        } else servoTimer.reset();
                        break;
                    }
                    break;
                case MOVING_TO_MIDDLE:
                    if ((int)servoTimer.milliseconds() >= robot.SERVO_STEP_TIME) { //step time elapsed
                        //do next step (in if header)
                        if (robot.stepServoTowardTarget(robot.cascadeOutputSystem.outputArmServo,robot.cascadeOutputSystem.ARM_EXTENDED_MIDDLE,robot.SERVO_STEP_SIZE)) {
                            //if at final position
                            outputArmServoMovementProcess = OutputArmServoMovementProcess.ENDING;
                        } else servoTimer.reset();
                        break;
                    }
                    break;
                case MOVING_TO_DOWN:
                    if ((int)servoTimer.milliseconds() >= robot.SERVO_STEP_TIME) { //step time elapsed
                        //do next step (in if header)
                        if (robot.stepServoTowardTarget(robot.cascadeOutputSystem.outputArmServo,robot.cascadeOutputSystem.ARM_EXTENDED_DOWN,robot.SERVO_STEP_SIZE)) {
                            //if at final position
                            outputArmServoMovementProcess = OutputArmServoMovementProcess.ENDING;
                        } else servoTimer.reset();
                        break;
                    }
                    break;
                case ENDING:
                    outputArmServoMovementProcess = OutputArmServoMovementProcess.AT_TARGET_POSITION;
                    break;
            }

            //telemetry on robot state
            runTelemetry(outputArmState, outputState, inputState, aState, bState);

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
    public void runTelemetry(OutputArmState outputArmState, IOState outputState, IOState inputState, AProcess aState, BProcess bState) {
        telemetry.addLine("---====STATES====---");
        telemetry.addData("Output Arm: ", outputArmState.toString());
        telemetry.addData("Output: ", outputState.toString());
        telemetry.addData("Input: ", inputState.toString());
        telemetry.addData("\nA-button process: ", aState.toString());
        telemetry.addData("B-button process: ", bState.toString());
        telemetry.update();
    }

}
