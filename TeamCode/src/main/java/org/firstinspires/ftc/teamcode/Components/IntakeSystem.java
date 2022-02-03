package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.LinkedList;
import java.util.List;

/**
 * This is NOT an opmode.
 * this class should contain the initialization for the intake system
 * as well as many of the methods the robot will use in its OpModes
 * @author Anthony Rubick
 */
public class IntakeSystem  implements Component {
    //TODO: encapsulation, and integrate the state enums from MecanumTeleOp.java
    ////////////////////////////// class variables //////////////////////////////
    //**info, measurements, known positions, etc.**//
    //arm positions
    public final double ARM_DOWN = 0.01;//0.34;//0.3; //0.05
    public final double ARM_UP = 0.7;//1.0; //.7
    public final double ARM_RAISED = 0.25;//0.55;//.2
    //grabber positions
    public final double GRABBER_FULL_OPEN = 0.3;
    public final double GRABBER_PARTIAL_OPEN = 0.55;
    public final double GRABBER_CLOSED = 0.72;

    /* --Public OpMode members.-- */
    public Servo intakeArmServo, intakeGrabberServo;
    /* --local OpMode members.-- */
    HardwareMap hwMap = null;

    ////////////////////////////// Constructors and init method //////////////////////////////
    /* --Constructors-- */
    public IntakeSystem(){}
    /* --Initialize standard Hardware interfaces-- */
    @Override
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        /*initialize hardware components*/
        // Define and initialize ALL installed servos.
        intakeArmServo = hwMap.get(Servo.class, "front_input_flipper_servo"); //down 0.01, up 0.7
        intakeGrabberServo = hwMap.get(Servo.class, "front_input_servo");//full open 0.3; half open 0.55; closed 0.72

        intakeArmServo.setPosition(ARM_DOWN);
        intakeGrabberServo.setPosition(GRABBER_FULL_OPEN);
    }
    /**
     * @return a list of all hardware devices included on the component
     */
    @Override
    public List<HardwareDevice> getAll() {
        List<HardwareDevice> hardwareDeviceList = new LinkedList<>();
        //add all hardware devices in component to hardwareDeviceList
        hardwareDeviceList.add(intakeGrabberServo);
        hardwareDeviceList.add(intakeArmServo);
        return hardwareDeviceList;
    }

    ////////////////////////////// Methods //////////////////////////////


    ////////////////////////////// Set Methods //////////////////////////////


    ////////////////////////////// Get Methods //////////////////////////////


}