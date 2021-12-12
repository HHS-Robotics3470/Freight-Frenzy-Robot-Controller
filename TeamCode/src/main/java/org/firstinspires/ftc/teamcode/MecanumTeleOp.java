package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="mecanum TeleOp", group="Competition")
public class MecanumTeleOp extends LinearOpMode {
    /*declare OpMode members, initialize some classes*/
    //an enum to represent output flipper states
    public enum OutputFlipperState {
        RETRACTED,
        FLAT,
        UP,
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
        ENDED
    }


    Hardware robot          = new Hardware();
    ElapsedTime runtime     = new ElapsedTime();
    ElapsedTime bProcessTimer = new ElapsedTime();
    ElapsedTime inputTimer = new ElapsedTime();

    //this is the control loop, basically the equivalent of a main function almost
    @Override
    public void runOpMode()
    {
        ////////////before driver presses play////////////
        //Variables
        //state enums
        OutputFlipperState outputFlipperState = OutputFlipperState.RETRACTED;
        IOState inputState = IOState.OPEN;
        IOState outputState = IOState.CLOSED;
        AProcess aState = AProcess.NOT_STARTED; //state of the process attached to the 'a' button
        BProcess bState = BProcess.NOT_STARTED; //state of the process attached to the 'b' button
        //button locks
        boolean aC, bC, yC, xC, upC, downC; //is currently pressed
        boolean aP = false, bP = false, yP = false, xP = false, upP = false, downP = false; //was previously pressed
        //other
        double tempOutFlipPos;
        int cascadeCount = 0;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        tempOutFlipPos = robot.params.get("cascadeFlipperServo").get("retracted"); //initialize this as the retracted position
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        bProcessTimer.reset();
        inputTimer.reset();


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
                Y: if output flipper is extended, retract, otherwise extend to flat

            D-Pad:
                Up: if output flipper is extended flat, go to extended up; if output flipper is extended down, go to extended flat; if output flipper is extended up, extend down
                Down: if output flipper is extended flat, go to extended down; if output flipper is extended up, go to extended flat; if output flipper is extended down, extend up
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
            double denom = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(r) , 1.0);
            robot.setDrivetrainPower(
                    (x-y-r)/denom,
                    (x+y+r)/denom,
                    (x-y+r)/denom,
                    (x+y-r)/denom
            ); //+12ms


            /////TRIGGERS AND BUMPERS/////

            /////ABXY/////
            //A input
            switch (aState) {
                case NOT_STARTED:
                    if (aC&&!aP) {
                        switch (inputState) {
                            case OPEN:
                                robot.frontInputServo.setPosition(
                                        robot.params.get("frontInputServo").get("closed")
                                ); //close
                                inputState = IOState.CLOSED;

                                //set up for rest of the states
                                aState = AProcess.IN_PROGRESS;
                                inputTimer.reset();
                                break;
                            case CLOSED:
                                robot.frontInputServo.setPosition(
                                        robot.params.get("frontInputServo").get("full open")
                                ); //open
                                inputState = IOState.OPEN;

                                //skip to end
                                robot.frontInputFlipperServo.setPosition(
                                        robot.params.get("frontInputFlipperServo").get("down")
                                ); //down
                                aState = AProcess.ENDED;
                                break;
                            default:
                                break;
                        }

                    }
                    break;
                case IN_PROGRESS:
                    //wait 400ms to give flipper time to raise / lower
                    if (inputTimer.seconds() >= 400) {
                        robot.frontInputFlipperServo.setPosition(
                                robot.params.get("frontInputFlipperServo").get("raised")
                        ); //raise slightly

                        aState = AProcess.ENDED;
                    }
                    break;
                case ENDED:
                default:
                    aState = AProcess.NOT_STARTED;
                    break;
            }
            //B transfer input to output
            //cascadeCount saves the position of the cascade so we can return to it later
            //tempOutFlipPos saves the position of the flipper so we can return to it later
            switch (bState) {
                case NOT_STARTED:
                    if (bC&&!bP) {
                        //save some info about the robots state so we can return to it at the end
                        tempOutFlipPos = robot.cascadeFlipperServo.getPosition();
                        cascadeCount = robot.cascadeLiftMotor.getCurrentPosition();

                        if (inputState != IOState.CLOSED) {
                            robot.frontInputServo.setPosition(
                                    robot.params.get("frontInputServo").get("closed")
                            ); //close input
                        }
                        robot.cascadeOutputServo.setPosition(
                                robot.params.get("cascadeOutputServo").get("receive")
                        ); //open output
                        robot.cascadeFlipperServo.setPosition(
                                robot.params.get("cascadeFlipperServo").get("retracted")
                        ); //fold back
                        //restrict relevant systems
                        inputState = IOState.RESTRICTED;
                        outputState = IOState.RESTRICTED;
                        outputFlipperState = OutputFlipperState.RESTRICTED;

                        //retract cascade
                        robot.cascadeLiftMotor.setPower(0);
                        robot.cascadeLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.cascadeLiftMotor.setTargetPosition(robot.params.get("cascadeLiftMotor").get("retracted").intValue());
                        robot.cascadeLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.cascadeLiftMotor.setPower(1);

                        bState = BProcess.STAGE_ONE; //go to next stage
                    }
                    break;
                case STAGE_ONE:
                    //wait for cascadeLiftMotor to finish moving
                    if (!robot.cascadeLiftMotor.isBusy()) {
                        //reset motor, get out of RUN_TO_POSITION mode
                        robot.cascadeLiftMotor.setPower(0);
                        robot.cascadeLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                        //raise front input flipper to drop off element
                        robot.frontInputFlipperServo.setPosition(
                                robot.params.get("frontInputFlipperServo").get("up")
                        );
                        bState = BProcess.STAGE_TWO; //go to next stage
                        bProcessTimer.reset();
                    }
                    break;
                case STAGE_TWO:
                    //wait 700ms for input flipper to raise fully
                    if (bProcessTimer.seconds() >= 700) {
                        robot.cascadeOutputServo.setPosition(
                                robot.params.get("cascadeOutputServo").get("closed")
                        ); //close output
                        robot.frontInputServo.setPosition(
                                robot.params.get("frontInputServo").get("half open")
                        ); //open input

                        bState = BProcess.STAGE_THREE; // go to next stage
                        bProcessTimer.reset();
                    }
                    break;
                case STAGE_THREE:
                    //wait 300ms to give time for the game element to transfer before re-extending cascade
                    if (bProcessTimer.seconds() >= 300) {
                        robot.frontInputFlipperServo.setPosition(
                                robot.params.get("frontInputFlipperServo").get("down")
                        );
                        robot.frontInputServo.setPosition(
                                robot.params.get("frontInputServo").get("full open")
                        ); //fully open input

                        //extend cascade back to previous position
                        robot.cascadeLiftMotor.setPower(0);
                        robot.cascadeLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.cascadeLiftMotor.setTargetPosition(cascadeCount);
                        robot.cascadeLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.cascadeLiftMotor.setPower(1);

                        bState = BProcess.ENDED; //go to next stage
                    }
                    break;
                case ENDED:
                    //wait for cascadeLiftMotor to finish moving
                    if (!robot.cascadeLiftMotor.isBusy()) {
                        //move output flipper back to level it was at previously
                        robot.cascadeFlipperServo.setPosition(tempOutFlipPos);

                        //reset motor, get out of RUN_TO_POSITION mode
                        robot.cascadeLiftMotor.setPower(0);
                        robot.cascadeLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                        //update robot state
                        outputState = IOState.CLOSED;
                        inputState = IOState.OPEN;
                        bState = BProcess.NOT_STARTED; //back to start
                    }
                    break;
                default:
                    bState = BProcess.NOT_STARTED; //back to start
                    break;
            }
            //X output
            switch (outputState) {
                case RESTRICTED:
                    if (xC&&!xP) {
                        robot.cascadeOutputServo.setPosition(
                                robot.params.get("cascadeOutputServo").get("closed")
                        ); //close
                        outputState = IOState.CLOSED;
                    }
                    break;
                case CLOSED:
                    if (xC&&!xP) {
                        robot.cascadeOutputServo.setPosition(
                                robot.params.get("cascadeOutputServo").get("drop")
                        );
                        outputState = IOState.OPEN;
                    }
                    break;
                default:
                    break;
            }
            //Y implemented later, bundled w/ UP and DOWN

            /////D-Pad/////
            //UP DOWN Y
            switch (outputFlipperState) {
                case RETRACTED:
                    if (yC&&!yP) {
                        robot.cascadeFlipperServo.setPosition(
                                robot.params.get("cascadeFlipperServo").get("flat")
                        ); //extend flat
                        outputFlipperState = OutputFlipperState.FLAT;
                    }
                    break;
                case FLAT:
                    if (upC&&!upP) {
                        robot.cascadeFlipperServo.setPosition(
                                robot.params.get("cascadeFlipperServo").get("up")
                        ); //up
                        outputFlipperState = OutputFlipperState.UP;
                    } else if (downC&&!downP) {
                        robot.cascadeFlipperServo.setPosition(
                                robot.params.get("cascadeFlipperServo").get("down")
                        ); //down
                        outputFlipperState = OutputFlipperState.DOWN;
                    } else if (yC&&!yP) {
                        robot.cascadeFlipperServo.setPosition(
                                robot.params.get("cascadeFlipperServo").get("retracted")
                        ); //retract
                        outputFlipperState = OutputFlipperState.RETRACTED;
                    }
                    break;
                case UP:
                    if (upC&&!upP) {
                        robot.cascadeFlipperServo.setPosition(
                                robot.params.get("cascadeFlipperServo").get("down")
                        ); //down
                        outputFlipperState = OutputFlipperState.DOWN;
                    } else if (downC&&!downP) {
                        robot.cascadeFlipperServo.setPosition(
                                robot.params.get("cascadeFlipperServo").get("flat")
                        ); //flat
                        outputFlipperState = OutputFlipperState.FLAT;
                    } else if (yC&&!yP) {
                        robot.cascadeFlipperServo.setPosition(
                                robot.params.get("cascadeFlipperServo").get("retracted")
                        ); //retract
                        outputFlipperState = OutputFlipperState.RETRACTED;
                    }
                    break;
                case DOWN:
                    if (upC&&!upP) {
                        robot.cascadeFlipperServo.setPosition(
                                robot.params.get("cascadeFlipperServo").get("flat")
                        ); //flat
                        outputFlipperState = OutputFlipperState.FLAT;
                    } else if (downC&&!downP) {
                        robot.cascadeFlipperServo.setPosition(
                                robot.params.get("cascadeFlipperServo").get("up")
                        ); //up
                        outputFlipperState = OutputFlipperState.UP;
                    } else if (yC&&!yP) {
                        robot.cascadeFlipperServo.setPosition(
                                robot.params.get("cascadeFlipperServo").get("retracted")
                        ); //retract
                        outputFlipperState = OutputFlipperState.RETRACTED;
                    }
                    break;
                case RESTRICTED:
                    break;
                default:
                    //should never be reached, output flipper state should never be null
                    robot.cascadeFlipperServo.setPosition(
                            robot.params.get("cascadeFlipperServo").get("retracted")
                    ); //retract
                    outputFlipperState = OutputFlipperState.RETRACTED;
                    break;
            }

            //noinspection StatementWithEmptyBody
            if (!bState.equals(BProcess.NOT_STARTED)) {
              //do nothing, this is here to disable the left and right buttons while the process attached to the 'b' button is happening
            }
            //LEFT
            else if (gamepad1.dpad_left) {
                if (robot.cascadeLiftMotor.getCurrentPosition() > robot.params.get("cascadeLiftMotor").get("extended").intValue()) {
                    robot.cascadeLiftMotor.setPower(0);
                    robot.runMotorToPosition(robot.cascadeLiftMotor, robot.params.get("cascadeLiftMotor").get("extended").intValue(), 1);
                } else {
                    robot.cascadeLiftMotor.setPower(1);
                }
            }
            //RIGHT
            else if (gamepad1.dpad_right) {
                if (robot.cascadeLiftMotor.getCurrentPosition() < robot.params.get("cascadeLiftMotor").get("retracted").intValue()) {
                    robot.cascadeLiftMotor.setPower(0);
                    robot.runMotorToPosition(robot.cascadeLiftMotor, robot.params.get("cascadeLiftMotor").get("retracted").intValue(), 1);
                } else {
                    robot.cascadeLiftMotor.setPower(-1);
                }
            } else robot.cascadeLiftMotor.setPower(0);

            //telemetry on robot state
            runTelemetry(outputFlipperState, outputState, inputState, aState, bState);

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
    public void runTelemetry(OutputFlipperState outputFlipperState, IOState outputState, IOState inputState, AProcess aState, BProcess bState) {
        telemetry.addLine("---====STATES====---");
        telemetry.addData("Output Flipper: ", outputFlipperState.toString());
        telemetry.addData("Output: ", outputState.toString());
        telemetry.addData("Input: ", inputState.toString());
        telemetry.addData("\nA-button process: ", aState.toString());
        telemetry.addData("B-button process: ", bState.toString());
        telemetry.update();
    }

}
