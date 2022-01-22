package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utility.Util;

import java.util.LinkedList;
import java.util.List;

public class CascadeOutputSystem implements Component {
    //TODO: encapsulation, and integrate the state enums from MecanumTeleOp.java
    ////////////////////////////// class variables //////////////////////////////
    //**info, measurements, known positions, etc.**//
    //cascade kit / linear extrusion
    public final int CASCADE_EXTENDED = 2150;
    public final int CASCADE_RETRACTED = 0;
    //arm
    public final double ARM_RETRACTED = .2;//0.15;
    public final double ARM_EXTENDED_FLAT = .79;//0.73;
    public final double ARM_EXTENDED_UP = .65;//0.6;
    public final double ARM_EXTENDED_DOWN = .93;//0.9;
    //grabber
    public final double GRABBER_RECEIVE = .31;
    public final double GRABBER_DROP = 0.75;
    public final double GRABBER_CLOSED = 0.92;
    //TODO: recalculate at full battery
    //PID info
    //CALCULATED:
    // p:
    // i:
    // d:
    // f:
    //Default:
    // p: 10
    // i: 3
    public PIDFCoefficients velocityPIDFCoefficients = new PIDFCoefficients(1.3884311033898306,0.13884311033898306,0,13.884311033898306); //using values from drivetrain rn
    public PIDFCoefficients positionPIDFCoefficients = new PIDFCoefficients(5,0,0,0); //TODO: initialize this once PIDF coefficients are known
    public int targetPositionTolerance = 25;

    /* --Public OpMode members.-- */
    public Servo outputGrabberServo, outputArmServo;
    public DcMotorEx cascadeLiftMotor;

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
        //SERVOS:
        outputGrabberServo = hwMap.get(Servo.class, "cascade_output_servo"); //closed 1, open to drop 0.75, open to receive 0.5
        outputArmServo = hwMap.get(Servo.class, "cascade_output_flipper_servo"); // retracted 0.125, extended flat 0.8, extended up 0.6, extended down 0.95
        outputGrabberServo.setPosition(GRABBER_CLOSED);
        outputArmServo.setPosition(ARM_RETRACTED);

        //MOTORS:
        /*
        cascadeLiftMotor = hwMap.get(DcMotorEx.class, "cascade_lift_motor");        //expansion hub port 1 //max extension = 2150 counts
        cascadeLiftMotor.resetDeviceConfigurationForOpMode();
        cascadeLiftMotor.setPower(0);
        cascadeLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cascadeLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cascadeLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cascadeLiftMotor.setDirection(DcMotor.Direction.FORWARD);
        //cascade PIDF stuff
        try {
            cascadeLiftMotor.setVelocityPIDFCoefficients(velocityPIDFCoefficients.p,velocityPIDFCoefficients.i,velocityPIDFCoefficients.d,velocityPIDFCoefficients.f);
            cascadeLiftMotor.setTargetPositionTolerance(targetPositionTolerance);
        } catch (UnsupportedOperationException e) {
            //if setting pid coefficients throws an error due to motor control algorithm being legacy, try to change the algorithm
            cascadeLiftMotor.getMotorType().getHubVelocityParams().algorithm = MotorControlAlgorithm.PIDF;
            //then try setting pid stuff again
            cascadeLiftMotor.setVelocityPIDFCoefficients(velocityPIDFCoefficients.p,velocityPIDFCoefficients.i,velocityPIDFCoefficients.d,velocityPIDFCoefficients.f);
            cascadeLiftMotor.setTargetPositionTolerance(targetPositionTolerance);
        }
        */
    }
    /**
     * @return a list of all hardware devices included on the component
     */
    @Override
    public List<HardwareDevice> getAll() {
        List<HardwareDevice> hardwareDeviceList = new LinkedList<>();
        //add all hardware devices in component to hardwareDeviceList
        hardwareDeviceList.add(cascadeLiftMotor);
        hardwareDeviceList.add(outputGrabberServo);
        hardwareDeviceList.add(outputArmServo);
        return hardwareDeviceList;
    }

    ////////////////////////////// Methods //////////////////////////////
    public void setCascadePower(double power) {
        cascadeLiftMotor.setPower(power);
    }
    public void extendCascadeToPosition(int targetPosition, double power) {
        //make sure target position is within bounds
        if (targetPosition > CASCADE_EXTENDED) targetPosition = CASCADE_EXTENDED;
        else if (targetPosition < CASCADE_RETRACTED) targetPosition = CASCADE_RETRACTED;
        //run motor to target position
        Util.runMotorToPosition(cascadeLiftMotor, targetPosition, power);
    }

    ////////////////////////////// Set Methods //////////////////////////////


    ////////////////////////////// Get Methods //////////////////////////////


}