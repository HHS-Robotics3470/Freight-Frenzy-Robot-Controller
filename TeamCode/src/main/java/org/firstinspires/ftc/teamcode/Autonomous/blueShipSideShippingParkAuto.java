package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Components.CascadeOutputSystem;
import org.firstinspires.ftc.teamcode.Hardware;

/**
 * copied version of BlueWarehouseSideAutonomous.java, changed up to work for the other side,
 * TODO: needs to be redone
 * we use this at the remote events
 * @author Anthony Rubick
 */
@Autonomous(name="blue ShippingSide ShippingPark", group="Needs Calibration" )
//@Disabled //this line disables the autonomous from appearing on the driver station, remove it for your code
public class blueShipSideShippingParkAuto extends org.firstinspires.ftc.teamcode.Autonomous.Autonomous {

    /*declare OpMode members, initialize some classes*/
    //Hardware robot          = new Hardware(); // moved to superclass
    //ElapsedTime runtime     = new ElapsedTime();

    //this is the control loop, basically the equivalent of a main function almost
    @Override
    public void runOpMode()
    {
        super.setStartPos(StartPos.BLUE_SHIPPING);
        ////////////before driver presses play////////////
        //Variables
        double pi = Math.PI;
        double movementSpeed = 0.8;
        //TODO: all of these need to be calibrated, measured, tested, etc.
        // currently they are all either guesses, or make nothing happen
        //level of the shipping hub that the preloaded freight needs to be moved to
        int level; // 0 == bottom, 1 == middle, 2 == top    see appendix D (pg 42) of gm2 for details
        //boolean[] barcodes = {true, true, true};   //is there a freight element in each barcode
        //index 0, closest to shipping hub
        //index 1, middle
        //index 2, farthest from shipping hub

        //distances, in meters, needed for specific movements
        double driveYStep1_2   = 0.9144;//m
        double dist1_3Base   = 0.4056;
        double driveXLevel0   = dist1_3Base;   //m
        double driveXLevel1   = dist1_3Base;
        double driveXLevel2   = dist1_3Base+.1;
        double driveXStep1_3   = 0.2032; //around 8 in

        double dist1_3;

        double rotateStep2_1 = pi/2.0; //90 degrees
        double rotateErrorStep2_1 = pi/12.0; //difference between desired rotation and actual rotation
        double driveYStep2_1 = 1.8288; //strafe to turntable
        double driveYStep2_2 = 0.4; //final touches, get right up to the turntable, slower
        double driveXStep2_2 = 0.25;
        int turntableTimeMS = 3000; //time, in ms, to turn the turntable

        double driveXStep3_1 = 0.3;

        //directions for various movements


        //distances, in encoder ticks, needed for specific movements

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.initVuforiaAndTfod(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        sleep(3000);
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        //runtime.reset();

        ////////////after driver presses play////////////

        //autonomous routine goes here step by step
        /*
        assuming we start in the spot closer to the warehouse:
        1) (freight) max 26pts
        6pts for delivering the pre-loaded box to the shipping hub + (10pts if we use the duck, 20pts if we use the team-shipping element (we choose if they use the duck or the team shipping element to determine the level))
        -using vuforia, determine which level the pre-loaded element needs to be put onto
        -move to the shipping hub
        -place the pre-load box onto the shipping hub, on the level previously determined

        2) turntable
        -strafe to the turntable
        -turn turntable

        3) (parking) max 10pts
        -park COMPLETELY in the warehouse closest to our alliance shipping hub (10pts)
         */

        // this one is assuming we start on the blue alliance, in the start position closed to the warehouse
        // currently made

        /*step 1*/
        //1.1
        sleep(1000); //give it time to find it
        level = super.levelByMarker();

        //1.2
        telemetry.addData("level: ", level);
        telemetry.addData("step: ",1.2);
        telemetry.update();
        //move to the shipping hub

        //y- movement to be in-line w/ shipping hub
        //robot facing: <=      needs to move:  ^
        robot.driveTrain.strafeToDistance(movementSpeed, 0, driveYStep1_2);

        //1.3
        telemetry.addData("level: ", level);
        telemetry.addData("step: ",1.3);
        telemetry.update();
        //drop off into shipping hub
        //extend output flipping arm to proper level
        switch (level) {
            case 1: //middle
                dist1_3 = driveXLevel1;
                robot.cascadeOutputSystem.moveArmToTarget(CascadeOutputSystem.OutputArmPosition.MIDDLE);
                //robot facing: <=      needs to move:  =>
                robot.driveTrain.strafeToDistance(movementSpeed, -pi/2, driveXLevel1);
                break;
            case 2: //top
                dist1_3 = driveXLevel2;
                //move servo to position
                robot.cascadeOutputSystem.moveArmToTarget(CascadeOutputSystem.OutputArmPosition.UP);
                //move forward a bit to reach the top thing
                //robot facing <=   needs to move =>
                robot.driveTrain.strafeToDistance(movementSpeed, -pi/2, driveXLevel2);
                robot.cascadeOutputSystem.outputGrabberServo.setPosition(robot.cascadeOutputSystem.GRABBER_RECEIVE);

                //open output all the way (receive) then let the normal opening effectively close it
                sleep(250);
                break;
            case 0: //bottom and default
            default:
                dist1_3 = driveXLevel0;
                //move servo to position
                robot.cascadeOutputSystem.moveArmToTarget(CascadeOutputSystem.OutputArmPosition.DOWN);
                //robot facing <=   needs to move =>
                robot.driveTrain.strafeToDistance(movementSpeed, -pi/2, driveXLevel0);
                break;
        }

        robot.intakeSystem.intakeArmServo.setPosition(robot.intakeSystem.ARM_RAISED);//raise the arm while in transit
        //drop off element
        robot.cascadeOutputSystem.outputGrabberServo.setPosition(robot.cascadeOutputSystem.GRABBER_DROP);
        sleep(250); //give time to move

        //robot facing <=   needs to move <=
        robot.driveTrain.strafeToDistance(movementSpeed, pi/2, driveXStep1_3);
        //sleep(500);

        //retract output flipping arm, and prep for movement
        //robot.intakeSystem.intakeArmServo.setPosition(robot.intakeSystem.ARM_RAISED);
        //robot.intakeSystem.intakeGrabberServo.setPosition(robot.intakeSystem.GRABBER_FULL_OPEN);
        //move servo to position
        robot.cascadeOutputSystem.moveArmToTarget(CascadeOutputSystem.OutputArmPosition.RETRACTED);

        /*step 2*/
        //step 2.1
        telemetry.addData("level: ", level);
        telemetry.addData("step: ",2.1);
        telemetry.update();
        //y movement to get over to turntable
        //robot facing: <=       needs to move:  v
        robot.driveTrain.strafeToDistance(movementSpeed, pi, driveYStep2_1);

        //rotate so wheel is in right place for rotation
        //robot facing: <-      needs to face:  V
        robot.driveTrain.rotateByAngle(rotateStep2_1-rotateErrorStep2_1, movementSpeed/3.0);

        //bump into wall
        //facing V moving =>
        robot.driveTrain.strafeToDistance(movementSpeed/2.0, pi, driveXStep2_2);
        //facing V moving V
        robot.driveTrain.strafeToDistance(movementSpeed/2.0, pi, driveYStep2_2);
        //final adjustments to turntable
        //facing V moving <=
        if (level == 2) {
            robot.driveTrain.strafeToDistance(movementSpeed / 2.0, 0, driveXStep2_2+0.1); //0.3
        }
        else {
            robot.driveTrain.strafeToDistance(movementSpeed / 2.0, 0, driveXStep2_2); //0.2
        }

        //step 2.2
        telemetry.addData("level: ", level);
        telemetry.addData("step: ",2.2);
        telemetry.update();

        //turn turntable
        robot.turntableMotor.setPower(Hardware.TURNTABLE_SPEED);
        //move back slowly while turning turntable to regain contact if its lost
        //facing V moving <=
        robot.driveTrain.strafeDirection(0.025, 0);
        //wait
        sleep(turntableTimeMS);
        //stop motors
        robot.driveTrain.setPower(0);
        robot.turntableMotor.setPower(0);
        sleep(500);

        //3.1
        telemetry.addData("level: ", level);
        telemetry.addData("step: ",3.1);
        telemetry.update();
        //park
        //robot facing: V      moving =>
        robot.driveTrain.strafeToDistance(movementSpeed,pi,driveXStep3_1);

        while (opModeIsActive()) {}
        ////////////after driver presses stop////////////

    }

    ////////////other methods and whatnot below here////////////

}