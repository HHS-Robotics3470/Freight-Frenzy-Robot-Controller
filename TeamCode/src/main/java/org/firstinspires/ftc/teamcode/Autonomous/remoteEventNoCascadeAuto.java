package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;

/**
 * copied version of BlueWarehouseSideAutonomous.java, changed up to work for the other side,
 * TODO: calibrate every distance, angle, etc. to be accurate
 * we use this at the remote events
 * @author Anthony Rubick
 */
@Autonomous(name="Red Warehouse-side Autonomous", group="Needs Calibration" )
//@Disabled //this line disables the autonomous from appearing on the driver station, remove it for your code
public class remoteEventNoCascadeAuto  extends org.firstinspires.ftc.teamcode.Autonomous.Autonomous {

    /*declare OpMode members, initialize some classes*/
    //Hardware robot          = new Hardware(); // moved to superclass
    ElapsedTime runtime     = new ElapsedTime();

    //this is the control loop, basically the equivalent of a main function almost
    @Override
    public void runOpMode()
    {
        super.setStartPos(StartPos.RED_WAREHOUSE);
        ////////////before driver presses play////////////
        //Variables
        double pi = Math.PI;
        //TODO: all of these need to be calibrated, measured, tested, etc.
        // currently they are all either guesses, or make nothing happen
        //level of the shipping hub that the preloaded freight needs to be moved to
        int level = 1; // 0 == bottom, 1 == middle, 2 == top    see appendix D (pg 42) of gm2 for details
        boolean[] barcodes = {true, true, true};   //is there a freight element in each barcode
        //index 0, closest to shipping hub
        //index 1, middle
        //index 2, farthest from shipping hub

        //distances, in meters, needed for specific movements
        double driveYStep1_2   = 0.6096;   //2 ft
        double driveXStep1_2   = 0.3556;   //1ft 2in
        double driveXStep1_3   = 0.15; //around 6 in

        double driveYStep2_1 = 1.8288;//6 ft

        //directions for various movements


        //distances, in encoder ticks, needed for specific movements
        int extendCascadeStep1_3 = robot.cascadeOutputSystem.CASCADE_EXTENDED;
        int retractCascadeStep1_3 = robot.cascadeOutputSystem.CASCADE_RETRACTED;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        super.robot.init(hardwareMap);
        super.robot.initVuforiaAndTfod(hardwareMap); //the super reference is redundant, just a reminder that the variable belongs to the superclass

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        ////////////after driver presses play////////////

        //autonomous routine goes here step by step
        /*
        assuming we start in the spot closer to the warehouse:
        1) (freight) max 26pts
        6pts for delivering the pre-loaded box to the shipping hub + (10pts if we use the duck, 20pts if we use the team-shipping element (we choose if they use the duck or the team shipping element to determine the level))
        -using vuforia, determine which level the pre-loaded element needs to be put onto
        -move to the shipping hub
        -place the pre-load box onto the shipping hub, on the level previously determined

        2) (parking) max 10pts
        -park COMPLETELY in the warehouse closest to our alliance shipping hub (10pts)

        TODO: future, grab freight from warehouse and put it in shipping hub
         */

        // this one is assuming we start on the blue alliance, in the start position closed to the warehouse
        // currently made

        /*step 1*/
        //1.1
        //todo: use vuforia to figure out which position the pre-loaded element needs to be delivered to, assign it to level, this will be done in the determineLevel() method of the Autonomous.java class that's in the same folder as this
        level = super.determineLevel(robot);

        //1.2
        //move to the shipping hub
        //y- movement to be in-line w/ shipping hub
        //robot facing: =>      needs to move:  V
        robot.driveTrain.strafeToDistance(1, 0, driveYStep1_2);
        //x- movement
        //robot facing: =>      needs to move:  <=
        robot.driveTrain.strafeToDistance(1, -pi/4.0, driveXStep1_2);


        robot.intakeSystem.intakeArmServo.setPosition(robot.intakeSystem.ARM_RAISED);//raise the arm while in transit

        //1.3
        //drop off into shipping hub
        //extend output flipping arm to proper level
        switch (level) {
            case 0: //bottom
                robot.cascadeOutputSystem.outputArmServo.setPosition(robot.cascadeOutputSystem.ARM_EXTENDED_DOWN);
                break;
            case 2: //top
                robot.cascadeOutputSystem.outputArmServo.setPosition(robot.cascadeOutputSystem.ARM_EXTENDED_UP);
                //move forward a bit to reach the top thing
                //robot facing =>   needs to move =>
                robot.driveTrain.strafeToDistance(0.5, pi/4, driveXStep1_3);
                break;
            case 1: //middle and default
            default:
                robot.cascadeOutputSystem.outputArmServo.setPosition(robot.cascadeOutputSystem.ARM_EXTENDED_FLAT);
                break;
        }
        sleep(300); //give time to move
        //drop off element
        robot.cascadeOutputSystem.outputGrabberServo.setPosition(robot.cascadeOutputSystem.GRABBER_DROP);
        sleep(100); //give time to move

        //retract output flipping arm, and prep for movement
        robot.cascadeOutputSystem.outputArmServo.setPosition(robot.cascadeOutputSystem.ARM_RETRACTED);
        robot.intakeSystem.intakeArmServo.setPosition(robot.intakeSystem.ARM_RAISED);
        robot.intakeSystem.intakeGrabberServo.setPosition(robot.intakeSystem.GRABBER_FULL_OPEN);

        /*step 2*/
        //2.1
        //strafe into the warehouse and end
        //rotate to face warehouse
        //robot facing: ->      needs to face:  ^
        robot.driveTrain.rotateByAngle(pi/4.0, 0.75);

        //y+ movement into warehouse
        //robot facing: ^       needs to move:  ^
        robot.driveTrain.strafeToDistance(1, pi/4.0, driveYStep2_1);

        ////////////after driver presses stop////////////
    }

    ////////////other methods and whatnot below here////////////

}