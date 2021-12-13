package org.firstinspires.ftc.teamcode.Templates;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Components.Component;

public class componentTemplate implements Component {
    //store info like motor specs, or servo position for a grabber to open/close
    enum Params {

    }

    ////////////////////////////// class variables //////////////////////////////
    /* --Public OpMode members.-- */

    /* --local OpMode members.-- */
    HardwareMap hwMap = null;

    ////////////////////////////// Constructors and init method //////////////////////////////
    /* --Constructors-- */

    /* --Initialize standard Hardware interfaces-- */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        /*initialize hardware components*/

    }

    ////////////////////////////// Methods //////////////////////////////


    ////////////////////////////// Set Methods //////////////////////////////


    ////////////////////////////// Get Methods //////////////////////////////


}