package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;

/**
 * this is a superclass that only has basic bits that all other auto routines will need
 * other routines will extend this class
 *
 * @author Anthony Rubick
 */
public abstract class Autonomous extends LinearOpMode {
    public enum StartPos {
        BLUE_WAREHOUSE,
        RED_WAREHOUSE,
        BLUE_SHIPPING,
        RED_SHIPPING
    }
    private StartPos startPos = null;
    public void setStartPos(StartPos startPos) {
        this.startPos = startPos;
    }
    Hardware robot          = new Hardware();

    //gameplan
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
    -park COMPLETELY in the warehouse closest to our alliance shipping hub (10pts) NOTE:

    autonomous steps:
    assuming we start in the spot closer to the carousel:
    1) (freight) max 26pts
    6pts for delivering the pre-loaded box to the shipping hub + (10pts if we use the duck, 20pts if we use the team-shipping element (we choose if they use the duck or the team shipping element to determine the level))
    -using vuforia, determine which level the pre-loaded element needs to be put onto
    -move to the shipping hub
    -place the pre-load box onto the shipping hub, on the level previously determined

    2) other freight, max 4 pts
    -move up to first piece of freight
    -collect
    -move equivalent distance as step 2.1, but opposite direction, back to in line w/ shipping hub
    -move over to shipping hub and deposit
    -move equivalent distance as step 2.4, but opposite direction
    -repeat 2.1-2.5 but for second piece of freight

    3) (carousel) max 10 pts
    -move to the corner w/ the carousel
    -spin the carousel until the duck comes off

    4) (parking) max 6pts
    -park COMPLETELY in the alliance storage unit (6pts) NOTE: one of the vu-marks is in the middle of the storage unit (on the wall)
     */



    //vuforia bits

    /**
     * depending on the startPos, use vuforia to figure out which position the pre-loaded element needs to be delivered to, assign it to level
     * @param robot the robot ... we need to access vuforia somehow
     * @return level, 0 == bottom, 1 == middle, 2 == top    see appendix D (pg 42) of gm2 for details
     */
    public int determineLevel(Hardware robot) {
        int level = 1; // 0 == bottom, 1 == middle, 2 == top    see appendix D (pg 42) of gm2 for details

        //depending on the startPos, use vuforia to figure out which position the pre-loaded element needs to be delivered to, assign it to level
        // for doing this, best option is to find where the 2 boxes are, and assume the team element is in the remaining space

        /*
        arranged in order of how the robot would see them
        + = barcode for top level (3)
        O = barcode for middle level (2)
        - = barcode for bottom level (1)
        S = shipping hub
        W = Warehouse / team shipping unit

        BLUE_WAREHOUSE
        W - O + S

        RED_WAREHOUSE
        S - O + W

        BLUE_SHIPPING
        S - O + W

        RED_SHIPPING
        W - O + S

         */

        //TODO: add logic to determine the level from the webcame using vuforia
        switch (startPos) {
            case BLUE_WAREHOUSE:
                //do logic here to find level
                break;
            case RED_WAREHOUSE:
                //do logic here to find level
                break;
            case BLUE_SHIPPING:
                //do logic here to find level
                break;
            case RED_SHIPPING:
                //do logic here to find level
                break;
            default:
                break;
        }

        return level;
    }

    //TODO: maybe something that uses vuforia to find the robots current position?

}
