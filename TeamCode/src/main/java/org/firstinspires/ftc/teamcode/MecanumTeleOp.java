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
        int cascadeCount = 0;
        double[] leftStickPolar;
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
                Up: if output flipper is extended flat, go to extended up; if output flipper is extended down, go to extended flat; if output flipper is retracted, extend flat
                Down: if output flipper is extended flat, go to extended down; if output flipper is extended up, go to extended flat; if output flipper is retracted, extend flat
                Left: extend cascade kit
                Right: retract cascade kit

             */

            /////JOYSTICKS/////
            leftStickPolar = Hardware.convertRectangularToPolar(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            if (Math.abs(gamepad1.right_stick_x) >= 0.2) {
                robot.rotate(gamepad1.right_stick_x); //rotate
            } else {
                robot.strafeDirection(leftStickPolar[0], leftStickPolar[1]); //strafe in direction that left joystick is pointing
            }

            /////TRIGGERS AND BUMPERS/////

            /////ABXY/////
            aC = gamepad1.a;
            bC = gamepad1.b;
            xC = gamepad1.x;
            yC = gamepad1.y;
            //A
            if (aC&&!aP) {
                robot.frontInputServo.setPosition((-1 * (robot.frontInputServo.getPosition() - 0.285) + 0.285) + 0.25); //toggle between 0.25 and .72
            }
            //B
            cascadeCount = robot.cascadeLiftMotor.getCurrentPosition();
            double outputFlipperPosition = robot.cascadeFlipperServo.getPosition();
            if (bC&&!bP) {
                robot.frontInputServo.setPosition(0.72); //close input
                robot.cascadeOutputServo.setPosition(0.5); //open output
                robot.cascadeFlipperServo.setPosition(0.125); //fold back
                robot.runMotorToPosition(robot.cascadeLiftMotor, 50, 1);//retract cascade

                robot.frontInputFlipperServo.setPosition(1);//TODO rotate input flipper to drop off position
                sleep(200);

                robot.frontInputServo.setPosition(0.55); //open input
                sleep(100);

                robot.frontInputFlipperServo.setPosition(0.75);//TODO rotate input flipper to collection position
                robot.frontInputServo.setPosition(0.25); //fully open input
                robot.cascadeOutputServo.setPosition(1); //close output
                robot.runMotorToPosition(robot.cascadeLiftMotor, cascadeCount, 1); //extend cascade back to previous position
                robot.cascadeFlipperServo.setPosition(outputFlipperPosition); //move output back to level it was at previously
                sleep(200);

            }
            //X
            if (xC&&!xP) {
                robot.cascadeOutputServo.setPosition((-1 * (robot.cascadeOutputServo.getPosition() - 0.125) + 0.125) + 0.75); //toggle between 0.75 and 1
            }
            //Y
            if (yC&&!yP) {
                if (robot.cascadeFlipperServo.getPosition() > 0.125 ){
                    robot.cascadeFlipperServo.setPosition(0.125);
                } else {
                    robot.cascadeFlipperServo.setPosition(0.8);
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

            }
            //DOWN
            if (downC&&!downP) {

            }
            upP = upC;
            downP = downC;
            //LEFT
            if (gamepad1.dpad_left) {
                if (robot.cascadeLiftMotor.getCurrentPosition() > 2150) {
                    robot.cascadeLiftMotor.setPower(0);
                    robot.runMotorToPosition(robot.cascadeLiftMotor, 2150, 1);
                } else {
                    robot.cascadeLiftMotor.setPower(1);
                }
            }
            //RIGHT
            else if (gamepad1.dpad_right) {
                if (robot.cascadeLiftMotor.getCurrentPosition() < 0) {
                    robot.cascadeLiftMotor.setPower(0);
                    robot.runMotorToPosition(robot.cascadeLiftMotor, 0, 1);
                } else {
                    robot.cascadeLiftMotor.setPower(-1);
                }
            } else robot.cascadeLiftMotor.setPower(0);


        }
        ////////////after driver presses stop////////////

    }

    ////////////other methods and whatnot below here////////////


}
