package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Hardware;

import java.util.List;

/**
 * this is a superclass that only has basic bits that all other auto routines will need
 * other routines will extend this class
 *
 * @author Anthony Rubick
 */
public abstract class Autonomous extends LinearOpMode {
    //DATA
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

    //game-plan
    /*
    autonomous steps:
    1) (freight) max 26pts
    6pts for delivering the pre-loaded box to the shipping hub + (10pts if we use the duck, 20pts if we use the team-shipping element (we choose if they use the duck or the team shipping element to determine the level))
    -using vuforia, determine which level the pre-loaded element needs to be put onto
    -move to the shipping hub
    -place the pre-load box onto the shipping hub, on the level previously determined

    2) turntable, max 10 pts NO TURNTABLE IN WAREHOUSE-SIDE START
    -strafe to the turntable
    -turn turntable

    if going to park in warehouse:
        -strafe back
        3) (parking) max 10pts
        -park COMPLETELY in the warehouse closest to our alliance shipping hub (10pts)
    else:
        4) (parking) max 6pts
        -park COMPLETELY in the alliance storage unit (6pts) NOTE: one of the vu-marks is in the middle of the storage unit (on the wall)
    */
    //auto naming conventions
    /*
    [alliance][barcode set][parking location]Auto

    [alliance]:
        -red        : red alliance
        -blue       : blue alliance

    [barcode set]:
        -WareSide   : barcode closer to warehouse
        -ShipSide   : barcode closer to team storage unit

    [parking location]:
        -WarehousePark  : park in the warehouse
        -StoragePark    : park in the team storage unit

     */


    ///Methods///

    //vuforia bits

    /**
     * depending on the startPos, use vuforia to figure out which position
     * the pre-loaded element needs to be delivered to, assign it to level
     * @return level, 0 == bottom, 1 == middle, 2 == top    see appendix D (pg 42) of gm2 for details
     */
    public int determineLevel() {
        int level = -1; // 0 == bottom, 1 == middle, 2 == top    see appendix D (pg 42) of gm2 for details

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
        //set up val1 val2 differently depending on start position
        double val1, val2;
        val1 = 245; //mm relative to camera
        val2 = 468; //mm relative to camera

        //detect the level differently depending on start position
        if (robot.tfod != null) {
            List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
                    if (!recognition.getLabel().equals("Marker")) {
                        double loc = (recognition.getLeft()+recognition.getRight())/2.0;
                        if (loc < val1){
                            level = 0;
                        }
                        else if (loc < val2 && loc > val1){
                            level = 1;
                        }
                        else if (loc > val2){
                            level = 2;
                        }
                        else {
                            level = -1;
                        }
                        //break;
                    }
                }
            }
        }

        return level;
    }

    /**
     * depending on the startPos, use vuforia to figure out which position
     * the pre-loaded element needs to be delivered to, assign it to level
     * @return level, 0 == bottom, 1 == middle, 2 == top    see appendix D (pg 42) of gm2 for details
     */
    public int levelByMarker() {
        int level = -1;
        double dist = 0.4;
        double speed = 0.4;
        double pi = Math.PI;
        if (markerVisible()) {
            switch (startPos) {
                case RED_WAREHOUSE:
                    //facing => moving V
                case BLUE_SHIPPING:
                    //facing <= moving ^
                    robot.driveTrain.strafeToDistance(speed, 0, dist);
                    sleep(500);
                    if (markerVisible()) {
                        level = 2;
                    } else level = 0;
                    robot.driveTrain.strafeToDistance(speed, pi, dist);
                    break;
                case BLUE_WAREHOUSE:
                    //facing <= moving V
                case RED_SHIPPING:
                    //facing => moving ^
                    robot.driveTrain.strafeToDistance(speed, pi, dist);
                    sleep(500);
                    if (markerVisible()) {
                        level = 0;
                    } else level = 2;
                    robot.driveTrain.strafeToDistance(speed, 0, dist);
                    break;
                default:
                    break;
            }
        } else level = 1;
        robot.driveTrain.strafeToDistance(0.4, pi/2.0, 0.1);
        return level;
    }

    /**
     * checks if a marker is in view
     * @return true if marker is visible, false if no markers are visible
     */
    public boolean markerVisible() {
        //look for a marker, return true if it's seen
        if (robot.tfod != null) {
            List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals("Marker")) {
                        return true;
                    }
                }
            }
        }
        return false;
    }
    //TODO: vuforia navigation

}
