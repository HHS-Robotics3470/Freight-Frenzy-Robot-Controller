package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Currently just psuedo code, comments\
 * next steps:
 * -code out comments with variables for any values like distance, power, or position
 * -through testing, find out what those variables need to be set to
 * -test and refine
 *
 * -basically, get one working perfectly, then copy paste and change some things for the other start positions
 * -create new code for every start position (red/blue, top/bottom)
 * -test and refine
 *
 * @author Anthony Rubick
 */
@Autonomous(name="Blue Warehouse-side Autonomous", group="Needs Calibration" )
@Disabled //this line disables the autonomous from appearing on the driver station, remove it for your code
public class BlueWarehouseSideAutonomous extends org.firstinspires.ftc.teamcode.Autonomous.Autonomous {

    /*declare OpMode members, initialize some classes*/
    //Hardware robot          = new Hardware(); // moved to superclass
    ElapsedTime runtime     = new ElapsedTime();

    //this is the control loop, basically the equivalent of a main function almost
    @Override
    public void runOpMode()
    {
        super.setStartPos(StartPos.BLUE_WAREHOUSE);
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
        double driveYMinusStep1_2   = 0.6096;   //2 ft
        double driveXPlusStep1_2    = 0.1524;   //6 in

        double driveXPlusStep2_1    = 0.254;    //10 in
        double driveYPlusStep2_1    = 0.3556;   //1ft 2in, added to during autonomous
        double distanceBetweenBarcodes = 0.21336;   //8.4 in
        double driveXStep2_2        = 0.0508;   //2 in
        double driveXStep2_4        = 0.0762;   //3 in


        double driveYPlusStep3_1 = 1.8288;//6 ft

        //directions for various movements


        //distances, in encoder ticks, needed for specific movements
        int extendCascadeStep1_3 = robot.cascadeOutputSystem.CASCADE_EXTENDED;
        int retractCascadeStep1_3 = robot.cascadeOutputSystem.CASCADE_RETRACTED;




        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.initVuforiaAndTfod(hardwareMap);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        ////////////after driver presses play////////////

        //autonomous routine goes here step by step
        //TODO: there is not frieght on the other barcodes, remove references to it
        /*
        assuming we start in the spot closer to the warehouse:
        1) (freight) max 26pts
        6pts for delivering the pre-loaded box to the shipping hub + (10pts if we use the duck, 20pts if we use the team-shipping element (we choose if they use the duck or the team shipping element to determine the level))
        -using vuforia, determine which level the pre-loaded element needs to be put onto
        -move to the shipping hub
        -place the pre-load box onto the shipping hub, on the level previously determined

        2) other freight, max 12 pts
        -move up to first piece of freight
        -collect
        -move equivalent distance as step 2.1, but opposite direction, back to in line w/ shipping hub
        -move over to shipping hub and deposit (6pts each time)
        -move equivalent distance as step 2.4, but opposite direction
        -repeat 2.1-2.5 but for second piece of freight


        3) (parking) max 10pts
        -park COMPLETELY in the warehouse closest to our alliance shipping hub (10pts)
         */

        // this one is assuming we start on the blue alliance, in the start position closed to the warehouse
        // currently made

        /*step 1*/
        //1.1
        //todo: use vuforia to figure out which position the pre-loaded element needs to be delivered to, assign it to level, this will be done in the determineLevel() method of the Autonomous.java class that's in the same folder as this
        level = super.determineLevel();
        switch (level) { //depending on the level, set one of the indexes in barcodes to false to represent the position of the game element
            //W - O + S
            case 0:
                barcodes[2] = false;
                break;
            case 2:
                barcodes[0] = false;
                break;
            case 1:
            default:
                barcodes[1] = false;
                break;
        }

        //1.2
        //move to the shipping hub
        //y- movement to be in-line w/ shipping hub
        //robot facing: <=      needs to move:  V
        robot.driveTrain.strafeToDistance(1, pi, driveYMinusStep1_2);
        //x+ movement if necessary (just make sure arm can extend long enough to drop off preloaded thing into shipping hub)
        //robot facing: <=      needs to move:  =>
        robot.driveTrain.strafeToDistance(1, -pi/4.0, driveXPlusStep1_2);


        robot.intakeSystem.intakeArmServo.setPosition(robot.intakeSystem.ARM_RAISED);//raise the arm while in transit

        //1.3
        //drop off into shipping hub
        //extend cascade kit
        robot.cascadeOutputSystem.extendCascadeToPosition(extendCascadeStep1_3, 1);

        //extend output flipping arm to proper level
        switch (level) {
            case 0: //bottom
                robot.cascadeOutputSystem.outputArmServo.setPosition(robot.cascadeOutputSystem.ARM_EXTENDED_DOWN);
                break;
            case 2: //top
                robot.cascadeOutputSystem.outputArmServo.setPosition(robot.cascadeOutputSystem.ARM_EXTENDED_UP);
                break;
            case 1: //middle and default
            default:
                robot.cascadeOutputSystem.outputArmServo.setPosition(robot.cascadeOutputSystem.ARM_EXTENDED_MIDDLE);
                break;
        }
        sleep(300); //give time to move
        //drop off element
        robot.cascadeOutputSystem.outputGrabberServo.setPosition(robot.cascadeOutputSystem.GRABBER_DROP);
        sleep(100); //give time to move

        //retract output flipping arm
        robot.cascadeOutputSystem.outputArmServo.setPosition(robot.cascadeOutputSystem.ARM_RETRACTED);

        //retract cascade kit
        robot.cascadeOutputSystem.extendCascadeToPosition(retractCascadeStep1_3, 1);

        /*step 2*/
        //pick up and deposit additional freight
        //2.1
        //x+ (maybe x- in future) movement until in-line with bar codes
        //robot facing: <=      needs to move:  =>
        robot.driveTrain.strafeToDistance(1, -pi/4.0, driveXPlusStep2_1);

        //rotate 180 degrees so webcam is facing x- direction
        //robot facing: <-      needs to face:  ->
        robot.driveTrain.rotateByAngle(pi, 0.75);

        for (int i = 0; i < 2; i++) { //do twice
            //2.1 continued
            //y+ movement to next piece of freight, //distance depends on variables i and level
            //robot facing: =>      needs to move:  ^
            int j = 0;
            for (j = 0; j < barcodes.length; j++) { //find closest block
                if (barcodes[j]) {
                    barcodes[j] = false;
                    break;
                }
            }
            robot.driveTrain.strafeToDistance(1, pi, driveYPlusStep2_1 +distanceBetweenBarcodes*j);

            //2.2
            robot.intakeSystem.intakeArmServo.setPosition(robot.intakeSystem.ARM_DOWN);
            robot.intakeSystem.intakeGrabberServo.setPosition(robot.intakeSystem.GRABBER_FULL_OPEN);
            sleep(300);
            //move forward, collect, then move back a bit
            //robot facing: =>      needs to move:  =>
            robot.driveTrain.strafeToDistance(1, pi/4.0, driveXStep2_2);

            //collect
            robot.intakeSystem.intakeGrabberServo.setPosition(robot.intakeSystem.GRABBER_CLOSED);
            sleep(100);

            robot.intakeSystem.intakeArmServo.setPosition(robot.intakeSystem.ARM_RAISED);
            sleep(300);

            //robot facing: =>      needs to move:  <=
            robot.driveTrain.strafeToDistance(1, -pi/4.0, driveXStep2_2);

            //2.3
            //y- movement, equal distance, opposite direction, as step 2.1.     until inline w/ shipping hub again
            //robot facing: =>      needs to move:  V
            robot.driveTrain.strafeToDistance(1, 0, driveYPlusStep2_1+distanceBetweenBarcodes*j);

            //2.4
            //move to shipping hub and deposit
            //x+ movement until up to shipping hub
            //robot facing: =>      needs to move:  =>
            robot.driveTrain.strafeToDistance(1,pi/4.0, driveXStep2_4);

            //drop off from input to bottom level
            robot.intakeSystem.intakeGrabberServo.setPosition(robot.intakeSystem.GRABBER_PARTIAL_OPEN);
            sleep(100);

            //2.5
            //move back to where we ended step 2.1
            //x- movement, equal distance, opposite direction as step 2.4
            //robot facing: =>      needs to move:  <=
            robot.driveTrain.strafeToDistance(1,-pi/4.0, driveXStep2_4);
            robot.intakeSystem.intakeArmServo.setPosition(robot.intakeSystem.ARM_RAISED);
        }//2.6, repeat

        robot.intakeSystem.intakeArmServo.setPosition(robot.intakeSystem.ARM_RAISED);
        robot.intakeSystem.intakeGrabberServo.setPosition(robot.intakeSystem.GRABBER_FULL_OPEN);

        /*step 3*/
        //3.1
        //strafe into the warehouse and end
        //rotate to face warehouse
        //robot facing: ->      needs to face:  ^
        robot.driveTrain.rotateByAngle(pi/4.0, 0.75);

        //y+ movement into warehouse
        //robot facing: ^       needs to move:  ^
        robot.driveTrain.strafeToDistance(1, pi/4.0, driveYPlusStep3_1);

        ////////////after driver presses stop////////////
    }

    ////////////other methods and whatnot below here////////////

}
