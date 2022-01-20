package org.firstinspires.ftc.teamcode.TestingTuningAndDemos.Demos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;

/**
 * demos / tests PID movement methods
 * @author Anthony Rubick
 */
@Autonomous(name = "PID Demo Auto", group = "Demos")
//@Disabled
public class PIDDemoAuto extends LinearOpMode {
    /*declare OpMode members, initialize some classes*/
    Hardware robot = new Hardware();
    ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        // declare some variables if needed

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //auto routine
        //uses current strafe method
        demoDriveTrainPID();

        //uses the strafe methods being tested
        //strafeToDistanceBuiltInPIDs(1, Math.PI, 0.2); //strafe left
        //strafeToDistanceBuiltInPIDs(1, 0, 0.4); //strafe right
        //strafeToDistanceBuiltInPIDs(1, 3 * Math.PI, 0.2); //strafe left
        //strafe in a rectangle
        //strafeToDistanceBuiltInPIDs(1, Math.PI / 4.0, 0.2);
        //strafeToDistanceBuiltInPIDs(0.5, Math.PI * 3.0 / 4.0, 0.3);
        //strafeToDistanceBuiltInPIDs(0.5, Math.PI * 5.0 / 4.0, 0.2);
        //strafeToDistanceBuiltInPIDs(1, -Math.PI / 4.0, 0.3);


        //strafe(1, 0, .1);
        //strafeToDistanceNoHeading(1,Math.PI,.2);
        //strafeToDistanceNoHeading(1, Math.PI/2.0, 0.1);
        //robot.strafeToDistance(0.3, 4.0*Math.PI/6.0, 0.2);
        //strafeToDistanceNoHeading(0.3, -7.0*Math.PI/6.0, 0.2);
        //robot.strafeToDistance(0.3, -10.0*Math.PI/6.0, 0.2);
    }
    private void demoDriveTrainPID() {
        double sqrt2 = Math.sqrt(2.0);
        double dist = 0.5;
        //strafe in a square centered on where it initialized
        robot.driveTrain.strafeToDistance(1, Math.PI/2.0, dist);sleep(400);                         //0      `|`                1,5
        robot.driveTrain.strafeToDistance(.2, 5.0*Math.PI/4.0, sqrt2*dist);sleep(400); //1      ./               /  |  \
        robot.driveTrain.strafeToDistance(.5, -Math.PI/4.0, sqrt2*dist);sleep(400);    //2      \.             /    |    \
        robot.driveTrain.strafeToDistance(1, Math.PI/4.0, sqrt2*dist);sleep(400);      //3      /`          2,6----0,7     4
        robot.driveTrain.strafeToDistance(1, -5.0*Math.PI/4.0, sqrt2*dist);sleep(400); //4      `\             \         /
        robot.driveTrain.strafeToDistance(.2, -3.0*Math.PI/4.0, sqrt2*dist);sleep(400);//5      ./               \     /
        robot.driveTrain.strafeToDistance(1, 0, dist);sleep(400);                                   //6      __.                 3

        /*
        movements
        0 y
        '|'
        1 y
        ./
        2 n
        ./
        3 y
        /'
        4 n
        _.
        /'
        5 n
        '|'
        \.
        6 y
        _.
         */
    }
    /**
     * causes the robot to strafe a given direction, at a given power, for a given distance using build in PID controllers
     * power and distance should always be positive
     *
     * @param power          power factor
     * @param angle          direction to strafe relative to robot, measured in radians, angle of 0 == starboard
     * @param targetDistance distance to strafe, measured in meters
     */
    public void strafeToDistanceBuiltInPIDs(double power, double angle, double targetDistance) {
        power = Math.abs(power);
        targetDistance = Math.abs(targetDistance);
        //DATA
        boolean moving = true;

        //calculate desired power for each diagonal motor pair
        double FrBlPairPower = Math.sin(angle - (Math.PI / 4)); //this is just done to help convert the radial inputs (magnitude and angle) into target distances that the PID can use
        double FlBrPairPower = Math.sin(angle + (Math.PI / 4));
        //find the desired target for each strafe PID
        int FrBlAxisTarget = (int) (targetDistance * (FrBlPairPower) * robot.NADO_COUNTS_PER_METER);
        int FlBrAxisTarget = (int) (targetDistance * (FlBrPairPower) * robot.NADO_COUNTS_PER_METER);
        //scale power appropriately
        FrBlPairPower *= power;
        FrBlPairPower *= power;

        //stop and prep
        robot.driveTrain.setPower(0);
        robot.driveTrain.driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.driveTrain.driveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.driveTrain.driveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.driveTrain.driveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set target
        robot.driveTrain.driveFrontRight.setTargetPosition(robot.driveTrain.driveFrontRight.getCurrentPosition() + FrBlAxisTarget);
        robot.driveTrain.driveFrontLeft.setTargetPosition(robot.driveTrain.driveFrontLeft.getCurrentPosition() + FlBrAxisTarget);
        robot.driveTrain.driveBackLeft.setTargetPosition(robot.driveTrain.driveBackLeft.getCurrentPosition() + FrBlAxisTarget);
        robot.driveTrain.driveBackRight.setTargetPosition(robot.driveTrain.driveBackRight.getCurrentPosition() + FlBrAxisTarget);

        //set to run to position
        robot.driveTrain.driveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.driveTrain.driveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.driveTrain.driveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.driveTrain.driveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set power
        robot.driveTrain.setPower(FrBlPairPower, FlBrPairPower);

        //let RUN_TO_POSITION PID do its thing
        while (moving && opModeIsActive()) {
            //are any of the motors busy?
            moving = robot.driveTrain.driveFrontRight.isBusy() || robot.driveTrain.driveFrontLeft.isBusy(); //save 6ms per loop by only reading from one motor of each diagonal pair
        }
        //give a little more time to settle
        sleep(500);

        //stop and go back to normal
        robot.driveTrain.setPower(0);
        robot.driveTrain.driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.driveTrain.driveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.driveTrain.driveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.driveTrain.driveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * causes the robot to strafe a given direction, at a given power, for a given distance using PIDs
     * power and distance should always be positive
     *
     * @param power          power factor
     * @param angle          direction to strafe relative to robot, measure in radians, angle of 0 == starboard
     * @param targetDistance distance to strafe, measured in meters
     */
    /*
    public void strafeToDistanceOld(double power, double angle, double targetDistance) {
        //DATA
        double rotationCorrection;
        //initial heading and encoder counts
        double initialHeading = robot.getGlobalAngle();
        int initialFrBlAxisEncoderCount = (robot.driveTrain.driveFrontRight.getCurrentPosition() + robot.driveTrain.driveBackLeft.getCurrentPosition()) / 2;
        int initialFlBrAxisEncoderCount = (robot.driveTrain.driveFrontLeft.getCurrentPosition() + robot.driveTrain.driveBackRight.getCurrentPosition()) / 2;

        //calculate desired power for each diagonal motor pair
        double FrBlPairPower = Math.sin(angle - (Math.PI / 4));
        double FlBrPairPower = Math.sin(angle + (Math.PI / 4));

        //find the desired target for each strafe PID
        int FrBlAxisTarget = (int) (targetDistance * (FrBlPairPower) * robot.NADO_COUNTS_PER_METER);
        int FlBrAxisTarget = (int) (targetDistance * (FlBrPairPower) * robot.NADO_COUNTS_PER_METER);

        //set up the PIDs
        power = Math.abs(power);
        //FrBl PID
        robot.FrBlStrafePIDController.reset();
        robot.FrBlStrafePIDController.setSetpoint(FrBlAxisTarget);
        robot.FrBlStrafePIDController.setOutputRange(-power * FrBlPairPower, power * FrBlPairPower);
        robot.FrBlStrafePIDController.enable(runtime.seconds());
        //FlBr PID
        robot.FlBrStrafePIDController.reset();
        robot.FlBrStrafePIDController.setSetpoint(FlBrAxisTarget);
        robot.FlBrStrafePIDController.setOutputRange(-power * FlBrPairPower, power * FlBrPairPower);
        robot.FlBrStrafePIDController.enable(runtime.seconds());
        //rotation PID (will be used to maintain constant heading)
        robot.rotatePIDController.reset();
        robot.rotatePIDController.setSetpoint(initialHeading);
        robot.rotatePIDController.setOutputRange(-power, power);
        robot.rotatePIDController.enable(runtime.seconds());


        while (!robot.FrBlStrafePIDController.onTarget() || !robot.FlBrStrafePIDController.onTarget()) {
            //calculate distance travelled by each diagonal pair
            int FrBlAxisEncoderCount = (robot.driveTrain.driveFrontRight.getCurrentPosition() + robot.driveTrain.driveBackLeft.getCurrentPosition()) / 2;
            int FlBrAxisEncoderCount = (robot.driveTrain.driveFrontLeft.getCurrentPosition() + robot.driveTrain.driveBackRight.getCurrentPosition()) / 2;

            //get correction values
            FrBlPairPower = robot.FrBlStrafePIDController.performPID(FrBlAxisEncoderCount - initialFrBlAxisEncoderCount, runtime.seconds());
            FlBrPairPower = robot.FlBrStrafePIDController.performPID(FlBrAxisEncoderCount - initialFlBrAxisEncoderCount, runtime.seconds());
            rotationCorrection = robot.rotatePIDController.performPID(robot.getGlobalAngle(), runtime.seconds());

            //largest power + rotation correction, or 1
            double denom = Math.max(
                    Math.max(Math.abs(FrBlPairPower), Math.abs(FlBrPairPower)) + Math.abs(rotationCorrection),
                    1.0
            );
            //assign drive motor powers
            robot.driveTrain.setPower(
                    (FrBlPairPower - rotationCorrection) / denom,
                    (FlBrPairPower + rotationCorrection) / denom,
                    (FrBlPairPower + rotationCorrection) / denom,
                    (FlBrPairPower - rotationCorrection) / denom
            );
        }
    }
    */
}