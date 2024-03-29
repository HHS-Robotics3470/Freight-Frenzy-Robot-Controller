package org.firstinspires.ftc.teamcode.Templates;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Components.Component;

import java.util.LinkedList;
import java.util.List;

/**
 * This is NOT an opmode.
 *
 * Template for components
 * @author Anthony Rubick
 */
public class componentTemplate implements Component {
    ////////////////////////////// class variables //////////////////////////////
    //**info, measurements, known positions, etc.**//

    /* --Public OpMode members.-- */

    /* --local OpMode members.-- */
    HardwareMap hwMap = null;

    ////////////////////////////// Constructors and init method //////////////////////////////
    /* --Constructors-- */

    /* --Initialize standard Hardware interfaces-- */
    @Override
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        /*initialize hardware components*/

    }
    /**
     * @return a list of all hardware devices included on the component
     */
    @Override
    public List<HardwareDevice> getAll() {
        List<HardwareDevice> hardwareDeviceList = new LinkedList<>();
        //add all hardware devices in component to hardwareDeviceList

        return hardwareDeviceList;
    }

    ////////////////////////////// Methods //////////////////////////////


    ////////////////////////////// Set Methods //////////////////////////////


    ////////////////////////////// Get Methods //////////////////////////////


}