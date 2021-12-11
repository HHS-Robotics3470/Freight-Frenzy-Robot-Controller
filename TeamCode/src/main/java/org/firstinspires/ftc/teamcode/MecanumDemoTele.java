package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Mecanum Demo TeleOp", group = "Demos")
@Disabled
public class MecanumDemoTele extends LinearOpMode {
    /*declare OpMode members, initialize some classes*/
    Hardware robot          = new Hardware();
    ElapsedTime runtime     = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        // declare some variables if needed
        double[] leftStickPolar;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //left joystick strafes, right joystick rotates
            leftStickPolar = Hardware.convertRectangularToPolar(gamepad1.left_stick_x, -gamepad1.left_stick_y);

            telemetry.addData("left stick x: ", gamepad1.left_stick_x);
            telemetry.addData("left stick y: ", gamepad1.left_stick_y);
            telemetry.addData("right stick x: ", gamepad1.right_stick_x);
            telemetry.addData("right stick y: ", gamepad1.right_stick_y);
            telemetry.update();


            //don't turn and strafe at the same time
            if (Math.abs(gamepad1.right_stick_x) >= 0.2) {
                robot.rotate(gamepad1.right_stick_x);
            } else {
                robot.strafeDirection(leftStickPolar[0], leftStickPolar[1]);
            }

        }
    }
}
