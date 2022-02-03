package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.MecanumTeleNoCascade;

import java.util.LinkedList;
import java.util.List;

public class CascadeOutputSystem implements Component {
    //TODO: encapsulation, and integrate the state enums from MecanumTeleOp.java
    ////////////////////////////// class variables //////////////////////////////
    public enum OutputArmPosition {
        RETRACTED,
        UP,
        MIDDLE,
        DOWN
    }

    //**info, measurements, known positions, etc.**//
    //cascade kit / linear extrusion
    public final int CASCADE_EXTENDED = 2150;
    public final int CASCADE_RETRACTED = 0;
    //arm
    public final double ARM_RETRACTED = 0.01;//.2;//0.15;
    public final double ARM_EXTENDED_UP = 0.52;//.65;//0.6;
    public final double ARM_EXTENDED_MIDDLE = 0.68;//.82;//0.73;
    public final double ARM_EXTENDED_DOWN = 0.78;//.93;//0.9;
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
    //public PIDFCoefficients velocityPIDFCoefficients = new PIDFCoefficients(1.3884311033898306,0.13884311033898306,0,13.884311033898306); //using values from drivetrain rn
    //public PIDFCoefficients positionPIDFCoefficients = new PIDFCoefficients(5,0,0,0); //TODO: initialize this once PIDF coefficients are known
    //public int targetPositionTolerance = 25;

    /* --Public OpMode members.-- */
    public Servo outputGrabberServo;
    private Servo outputArmServo; //private so that it can only be controlled with stepper methods
    //public DcMotorEx cascadeLiftMotor;
    private OutputArmPosition outputArmPosition;
    private OutputArmPosition outputArmTarget;
    public ElapsedTime servoTimer;


    /* --local OpMode members.-- */
    HardwareMap hwMap = null;

    ////////////////////////////// Constructors and init method //////////////////////////////
    /* --Constructors-- */

    /* --Initialize standard Hardware interfaces-- */
    @Override
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //timer and state stuff
        servoTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        servoTimer.reset();
        outputArmPosition = OutputArmPosition.RETRACTED;
        outputArmTarget = OutputArmPosition.RETRACTED;

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
        //hardwareDeviceList.add(cascadeLiftMotor);
        hardwareDeviceList.add(outputGrabberServo);
        hardwareDeviceList.add(outputArmServo);
        return hardwareDeviceList;
    }

    ////////////////////////////// Methods //////////////////////////////

    /**
     * moves the servo toward a previously set target by step, each delayed by stepTime
     */
    public void moveArmToTarget() {
        moveArmToTarget(outputArmTarget);
    }
    /**
     * moves the servo toward target by step, each delayed by stepTime
     * @param target target position, an enum
     */
    public void moveArmToTarget(OutputArmPosition target) {
        while (!stepArmTowardTarget(target)) {
            //do nothing, wait
        }
    }


    /**
     * steps servo toward a previously set target by 1 step, used in a loop like so: while (!stepArmTowardTarget()) {}
     * use the setTarget() method to set the target
     * @return true if at target position, false otherwise or if step time (defined in Hardware.java) hasn't elapsed since last call
     */
    public boolean stepArmTowardTarget() {
        return stepArmTowardTarget(outputArmTarget);
    }
    /**
     * steps servo toward the target by 1 step, used in a loop like so:
     * //in game loop
     * while (!stepArmTowardTarget(target)) {}
     * @param targetArmPosition target position
     * @return true if at target position, false otherwise or if step time (defined in Hardware.java) hasn't elapsed since last call
     */
    public boolean stepArmTowardTarget(OutputArmPosition targetArmPosition) {
        double target;
        //translate passed targetPosition to corresponding position values
        switch (targetArmPosition) {
            case UP: target = ARM_EXTENDED_UP;break;
            case MIDDLE: target = ARM_EXTENDED_MIDDLE;break;
            case DOWN: target = ARM_EXTENDED_DOWN;break;
            case RETRACTED:
            default: target = ARM_RETRACTED; break;
        }

        if ((int)servoTimer.milliseconds() >= Hardware.SERVO_STEP_TIME) { //step time elapsed since last call
            //do next step (in if header)
            if (Hardware.stepServoTowardTarget(outputArmServo,target,Hardware.SERVO_STEP_SIZE)) {
                //if at final position
                outputArmPosition = targetArmPosition;

                return true;
            } else servoTimer.reset(); //reset timer for next step
        }
        return false;
    }

    /*
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
    */
    ////////////////////////////// Set Methods //////////////////////////////

    /**
     * @param outputArmTarget new target state for the servo
     */
    public void setOutputArmTarget(OutputArmPosition outputArmTarget) {
        this.outputArmTarget = outputArmTarget;
    }

    ////////////////////////////// Get Methods //////////////////////////////

    /**
     * @return outputArmTarget, target state of the servo
     */
    public OutputArmPosition getOutputArmTarget() {
        return outputArmTarget;
    }

    /**
     * @return outputArmPosition, current state of the servo
     */
    public OutputArmPosition getOutputArmPosition() {
        return outputArmPosition;
    }
}