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
    int ringsLeft = 4; //TODO update this
    int gameLengthSeconds = 3*60; //give them 3 minutes to play

    //button locks
    boolean aButtonCurPressed;
    boolean aButtonPrevPressed=false;


    //this is the control loop, basically the equivalent of a main function almost
    @Override
    public void runOpMode()
    {
        ////////////before driver presses play////////////
        //Variables
        //these variables control the max power of various components
        double flywheelSpeed = 1.0;
        double elevatorSpeed = 0.25;
        double rotationSpeed = 0.25;
        double drivePowerMultiplier = 0.5;

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

            right/left trigger/bumper control left/right forward/backward movement respectively
            //thumbsticks will control movement
            */
            //////ABXY bindings/////
            //fire turret
            aButtonCurPressed = gamepad1.a;
            if (aButtonCurPressed && !aButtonPrevPressed) {
                //reload
                robot.turretLauncher.setPosition(robot.LAUNCHER_RELOAD_POS);

                //if above stable elevation, go to neutral to allow it to reload properly
                if (currentTurretPitch < Math.toRadians(30)) {
                    //move to neutral elevation
                    robot.runMotorToPosition(
                            robot.turretElevator,
                            0,
                            elevatorSpeed
                    );

                    //wait, then go back to normal
                    sleep(200);

                    //move back
                    robot.runMotorToPosition(
                            robot.turretElevator,
                            (int)(currentTurretPitch/robot.GO_BILDA_RADIANS_PER_COUNTS),
                            elevatorSpeed
                    );
                }

                //turn on the fly wheels
                robot.flyWheel1.setPower(flywheelSpeed);
                robot.flyWheel2.setPower(flywheelSpeed);
                sleep(700);

                //make the launch servo push the ring into the fly wheels
                robot.turretLauncher.setPosition(robot.LAUNCHER_FIRE_POS);
                sleep(300);

                //turn off the fly wheels
                robot.flyWheel1.setPower(0);
                robot.flyWheel2.setPower(0);

                //update ring count
                ringsLeft --;

            }
            aButtonPrevPressed = aButtonCurPressed;

            //////D-Pad bindings/////
            //elevate and lower (max is 30degrees, min is 0)
            //30 degrees for the turret may need to be 55 degrees for the elevator
            currentTurretPitch=robot.turretElevator.getCurrentPosition() * robot.GO_BILDA_RADIANS_PER_COUNTS;
            if (gamepad1.dpad_up && currentTurretPitch < Math.toRadians(50)) {
                //elevate turret (+ power)
                robot.turretElevator.setPower(elevatorSpeed);
            }
            else if (gamepad1.dpad_down && currentTurretPitch > 0) {
                //lower turret (- power)
                robot.turretElevator.setPower(-elevatorSpeed);
            }
            else {
                //stop elevation
                robot.turretElevator.setPower(0);
            }

            //rotate (max is 135, min is -135 (basically, don't let then wrap up the wires by repeatedly rotating the turret in one direction
            currentTurretHeading=robot.turretRotator.getCurrentPosition() * robot.CORE_HEX_RADIANS_PER_COUNTS;
            if (gamepad1.dpad_left && currentTurretHeading > -0.75 * Math.PI) {
                //rotate the turret to the left (- power), if that wouldn't put it past the max distance
                robot.turretRotator.setPower(-rotationSpeed);
            }
            else if (gamepad1.dpad_right && currentTurretHeading < 0.75 * Math.PI) {
                //rotate the turret to the right (+ power)
                robot.turretRotator.setPower(rotationSpeed);
            }
            else {
                //stop the rotation
                robot.turretRotator.setPower(0);
            }

            //////thumbstick bindings/////
            /*tankControls(
                    -gamepad1.right_stick_y * drivePowerMultiplier,//negative bc the y-axis of the thumbsticks is inverted//gamepad1.right_bumper ? -0.2 : gamepad1.right_trigger,
                    -gamepad1.left_stick_y * drivePowerMultiplier//negative bc the y-axis of the thumbsticks is inverted//gamepad1.left_bumper ? -0.2 : gamepad1.left_trigger
            );*/
            tankControls(
                     -drivePowerMultiplier*( (gamepad1.right_stick_y>0) ? ( (Math.pow(4,gamepad1.right_stick_y)-1)/3.0 ) : ( -(Math.pow(4,-gamepad1.right_stick_y)-1)/3.0) ),
                    -drivePowerMultiplier*( (gamepad1.left_stick_y >0) ? ( (Math.pow(4,gamepad1.left_stick_y )-1)/3.0 ) : ( -(Math.pow(4,-gamepad1.left_stick_y )-1)/3.0) )
            );
            /*//temp diagnostics
            telemetry.addData("turret heading", Math.toDegrees(currentTurretHeading));
            telemetry.addData("turret pitch", Math.toDegrees(currentTurretPitch));
            telemetry.addData("elevation encoder count", robot.turretElevator.getCurrentPosition());
            */

            //if the user presses the back button, end the loop
            if (gamepad1.back) break;


            //tell the user how many rings, and how much time they have remaining
            telemetry.addData("rings remaining: ", ringsLeft);
            telemetry.addData("time remaining: ", (double)gameLengthSeconds - runtime.seconds());
            telemetry.update();
        }

        ////////////after driver presses stop////////////
        //tell the user that the game is over
        telemetry.clearAll();
        telemetry.addLine("GAME OVER");
        telemetry.addLine("thanks for playing :D");
        telemetry.update();

        //put all the turret stuff back to default positions
        robot.runMotorToPosition(robot.turretRotator, 0, rotationSpeed);
        robot.runMotorToPosition(robot.turretElevator,0, elevatorSpeed);
        robot.turretLauncher.setPosition(robot.LAUNCHER_FIRE_POS);
        tankControls(0,0);

        //sleep for 5 seconds
        sleep(5000);
    }

    ////////////other methods and whatnot below here////////////
    public void tankControls(double left, double right){
        //only care about the tenths digit (ex: if left = 0.9742, after this it'll equal 0.9)
        left -= left % 0.1;
        right -= right % 0.1;

        //assign movement
        robot.leftDrive.setPower(left);
        robot.rightDrive.setPower(right);

    }
}

