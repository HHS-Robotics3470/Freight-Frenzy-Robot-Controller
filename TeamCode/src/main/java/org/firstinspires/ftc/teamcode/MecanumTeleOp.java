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
        int outputFlipperState = 0; //0-3, 0=retracked, 1=extended flat, 2=extended up, 3=extended down
        boolean inputOpen = true;
        boolean outputOpen = false;
        int cascadeCount;
        //double[] leftStickPolar;
        //button locks
        boolean aC, bC, yC, xC, upC, downC; //is currently pressed
        boolean aP = false, bP = false, yP = false, xP = false, upP = false, downP = false; //was previously pressed


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

            /////JOYSTICKS/////
            //this has it in 2 steps, we could combine it to one and save ~4 calls to the math library (2 max, 2 abs)
            //leftStickPolar = Hardware.convertRectangularToPolar(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            //robot.strafeDirection(leftStickPolar[0], leftStickPolar[1]);
            //robot.rotateByCorrection(gamepad1.right_stick_x);
            //combined to 1 step, and optimized to minimize calls to the Math library, and eliminates need for trig functions
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double r = gamepad1.right_stick_x/2.0;
            double denom = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(r) , 1.0);
            robot.setDrivetrainPower(
                    (x-y-r)/denom,
                    (x+y+r)/denom,
                    (x-y+r)/denom,
                    (x+y-r)/denom
            );


            /////TRIGGERS AND BUMPERS/////

            /////ABXY/////
            aC = gamepad1.a;
            bC = gamepad1.b;
            xC = gamepad1.x;
            yC = gamepad1.y;
            //A
            if (aC&&!aP) {
                //robot.frontInputServo.setPosition((-1 * (robot.frontInputServo.getPosition() - 0.285) + 0.285) + 0.25); //toggle between 0.25 and .72
                if (inputOpen) { // if open
                    robot.frontInputServo.setPosition(
                            robot.params.get("frontInputServo").get("closed")
                    ); //close
                    sleep(400);
                    robot.frontInputFlipperServo.setPosition(
                            robot.params.get("frontInputFlipperServo").get("raised")
                    ); //raise slightly
                    inputOpen = false;
                } else { //else
                    robot.frontInputServo.setPosition(
                            robot.params.get("frontInputServo").get("full open")
                    ); //open
                    robot.frontInputFlipperServo.setPosition(
                            robot.params.get("frontInputFlipperServo").get("down")
                    ); //down
                    inputOpen = true;
                }
            }
            //B
            cascadeCount = robot.cascadeLiftMotor.getCurrentPosition();
            double outputFlipperPosition = robot.cascadeFlipperServo.getPosition();
            if (bC&&!bP) {
                robot.frontInputServo.setPosition(
                        robot.params.get("frontInputServo").get("closed")
                ); //close input
                robot.cascadeOutputServo.setPosition(
                        robot.params.get("cascadeOutputServo").get("receive")
                ); //open output
                robot.cascadeFlipperServo.setPosition(
                        robot.params.get("cascadeFlipperServo").get("retracted")
                ); //fold back
                robot.runMotorToPosition(robot.cascadeLiftMotor, robot.params.get("cascadeLiftMotor").get("retracted").intValue(), 1);//retract cascade

                robot.frontInputFlipperServo.setPosition(
                        robot.params.get("frontInputFlipperServo").get("up")
                );
                sleep(700);

                robot.cascadeOutputServo.setPosition(
                        robot.params.get("cascadeOutputServo").get("closed")
                ); //close output
                outputOpen = false;
                robot.frontInputServo.setPosition(
                        robot.params.get("frontInputServo").get("half open")
                ); //open input
                sleep(300);

                robot.frontInputFlipperServo.setPosition(
                        robot.params.get("frontInputFlipperServo").get("down")
                );
                robot.frontInputServo.setPosition(
                        robot.params.get("frontInputServo").get("full open")
                ); //fully open input
                inputOpen = true;
                robot.runMotorToPosition(robot.cascadeLiftMotor, cascadeCount, 1); //extend cascade back to previous position
                robot.cascadeFlipperServo.setPosition(outputFlipperPosition); //move output back to level it was at previously
                //sleep(200);
            }
            //X
            if (xC&&!xP) {
                if (outputOpen) { //if open
                    robot.cascadeOutputServo.setPosition(
                            robot.params.get("cascadeOutputServo").get("closed")
                    ); //close
                    outputOpen = false;
                } else {
                    robot.cascadeOutputServo.setPosition(
                            robot.params.get("cascadeOutputServo").get("drop")
                    );
                    outputOpen = true;
                }
            }
            //Y
            if (yC&&!yP) {
                if (outputFlipperState != 0){ //not retracted
                    robot.cascadeFlipperServo.setPosition(
                            robot.params.get("cascadeFlipperServo").get("retracted")
                    ); //retract
                    outputFlipperState=0;
                } else {
                    robot.cascadeFlipperServo.setPosition(
                            robot.params.get("cascadeFlipperServo").get("flat")
                    ); //extend flat
                    outputFlipperState=1;
                }
            }
            aP=aC;
            bP=bC;
            xP=xC;
            yP=yC;
            /////D-Pad/////
            upC = gamepad1.dpad_up;
            downC = gamepad1.dpad_down;
            //UP
            if (upC&&!upP) {
                if(outputFlipperState == 1) { //extended flat
                    robot.cascadeFlipperServo.setPosition(
                            robot.params.get("cascadeFlipperServo").get("up")
                    ); //up
                    outputFlipperState=2;
                } else if (outputFlipperState == 3) { //extended down
                    robot.cascadeFlipperServo.setPosition(
                            robot.params.get("cascadeFlipperServo").get("flat")
                    ); //flat
                    outputFlipperState=1;
                } else if (outputFlipperState == 2) { //extended up
                    robot.cascadeFlipperServo.setPosition(
                            robot.params.get("cascadeFlipperServo").get("down")
                    ); //down
                    outputFlipperState=3;
                }
            }
            //DOWN
            if (downC&&!downP) {
                if(outputFlipperState == 1) { //extended flat
                    robot.cascadeFlipperServo.setPosition(
                            robot.params.get("cascadeFlipperServo").get("down")
                    ); //down
                    outputFlipperState=3;
                } else if (outputFlipperState == 2) { //extended up
                    robot.cascadeFlipperServo.setPosition(
                            robot.params.get("cascadeFlipperServo").get("flat")
                    ); //flat
                    outputFlipperState=1;
                } else if (outputFlipperState == 3) { //extended down
                    robot.cascadeFlipperServo.setPosition(
                            robot.params.get("cascadeFlipperServo").get("up")
                    ); //up
                    outputFlipperState=2;
                }
            }
            upP = upC;
            downP = downC;
            //LEFT
            if (gamepad1.dpad_left) {
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


        }
        ////////////after driver presses stop////////////

    }

    ////////////other methods and whatnot below here////////////


}
