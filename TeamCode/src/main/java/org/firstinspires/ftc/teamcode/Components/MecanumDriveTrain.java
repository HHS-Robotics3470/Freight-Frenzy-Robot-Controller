package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.LinkedList;
import java.util.List;

/**
 * This is NOT an opmode.
 * this class should contain the initialization for the drive train,
 * as well as many of the methods the robot will use in its OpModes
 * @author Anthony Rubick
 */
public class MecanumDriveTrain implements Component {
    ////////////////////////////// class variables //////////////////////////////
    //**info, measurements, known positions, etc.**//
    // stats for the TorqueNADO motors
    public final double NADO_COUNTS_PER_MOTOR_REV = 1440;
    public final double NADO_DRIVE_GEAR_REDUCTION = 1;  // This is < 1.0 if geared UP (to increase speed)
    public final double NADO_WHEEL_DIAMETER_METERS= 0.2032; //(8") For figuring circumference
    public final double NADO_COUNTS_PER_METER      = (NADO_COUNTS_PER_MOTOR_REV * NADO_DRIVE_GEAR_REDUCTION) /
            (NADO_WHEEL_DIAMETER_METERS * Math.PI);
    public final double NADO_METERS_PER_COUNT = 1.0 / NADO_COUNTS_PER_METER;
    //PID info
    public PIDFCoefficients velocityPIDFCoefficients = new PIDFCoefficients(); //TODO: initialize this once PIDF coefficients are known
    public PIDFCoefficients positionPIDFCoefficients = new PIDFCoefficients(); //TODO: initialize this once PIDF coefficients are known
    public int targetPositionTolerance = 50;

    /* --Public OpMode members.-- */
    public DcMotorEx driveFrontRight,driveFrontLeft,driveBackLeft,driveBackRight; //drive motors

    /* --local OpMode members.-- */
    HardwareMap hwMap = null;

    ////////////////////////////// Constructors and init method //////////////////////////////
    /* --Constructors-- */
    public MecanumDriveTrain(){}
    /* --Initialize standard Hardware interfaces-- */
    @Override
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        /*initialize hardware components*/
        // Define and initialize all Motors
        driveFrontLeft  = hwMap.get(DcMotorEx.class, "front_left_drive");  //main hub port 0
        driveFrontRight = hwMap.get(DcMotorEx.class, "front_right_drive"); //main hub port 1
        driveBackRight  = hwMap.get(DcMotorEx.class, "back_right_drive");  //main hub port 2
        driveBackLeft   = hwMap.get(DcMotorEx.class, "back_left_drive");   //main hub port 3

        // Set all motors to zero power
        setPower(0.0);
        //reset all motors
        driveFrontRight.resetDeviceConfigurationForOpMode();
        driveFrontLeft.resetDeviceConfigurationForOpMode();
        driveBackLeft.resetDeviceConfigurationForOpMode();
        driveBackRight.resetDeviceConfigurationForOpMode();

        // Set run modes
        //TODO: set drive motors to RUN_WITHOUT_ENCODER when PIDs are integrated for movement in order to improve output power
        //reset encoders
        driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //run using encoders
        driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //torqueNADO motor
        driveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  //torqueNADO motor
        driveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);   //torqueNADO motor
        driveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  //torqueNADO motor
        //run without encoders

        // Set Zero power behavior
        driveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        driveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        driveBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        driveBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //assign motor directions
        driveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        driveFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        driveBackLeft.setDirection(DcMotor.Direction.FORWARD);
        driveBackRight.setDirection(DcMotor.Direction.REVERSE);

        //PIDF setup
        setVelocityPIDFCoefficients(velocityPIDFCoefficients);
        setPositionPIDFCoefficients(5);
        setTargetPositionTolerance(targetPositionTolerance);
    }

    ////////////////////////////// Methods //////////////////////////////
    /**
     * given a power and direction, causes the robot to strafe continuously in that given direction at the given power
     * @param power power factor
     * @param angle direction to strafe relative to robot, measure in radians, angle of 0 == starboard
     */
    public void strafeDirection(double power, double angle) {
        // calculate the power that needs to be assigned to each diagonal pair of motors
        double FrBlPairPower = Math.sin(angle - (Math.PI/4)) * power;
        double FlBrPairPower = Math.sin(angle + (Math.PI/4)) * power;
        setPower(FrBlPairPower, FlBrPairPower);
    }
    /**
     * TODO: reimplement heading correction once odometry is up and running
     * causes the robot to strafe a given direction, at a given power, for a given distance using PIDs
     * power and distance should always be positive
     * DOES NOT USE IMU ANYMORE BC THAT WAS CAUSING ISSUES
     * @param power             power factor
     * @param angle             direction to strafe relative to robot, measure in radians, angle of 0 == starboard
     * @param targetDistance    distance to strafe, measured in meters
     */
    public void strafeToDistance(double power, double angle, double targetDistance) {
        power = Math.abs(power);
        targetDistance = Math.abs(targetDistance);
        //DATA
        boolean moving = true;

        //calculate desired power for each diagonal motor pair
        double FrBlPairPower = Math.sin(angle - (Math.PI/4)); //this is just done to help convert the radial inputs (magnitude and angle) into target distances that the PID can use
        double FlBrPairPower = Math.sin(angle + (Math.PI/4));
        //find the desired target for each strafe PID
        int FrBlAxisTarget = (int) (targetDistance * (FrBlPairPower) * NADO_COUNTS_PER_METER);
        int FlBrAxisTarget = (int) (targetDistance * (FlBrPairPower) * NADO_COUNTS_PER_METER);
        //scale power appropriately
        FrBlPairPower *= power;
        FrBlPairPower *= power;

        //stop and prep
        setPower(0);
        driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set target
        driveFrontRight.setTargetPosition(driveFrontRight.getCurrentPosition()+FrBlAxisTarget);
        driveFrontLeft.setTargetPosition(driveFrontLeft.getCurrentPosition()+FlBrAxisTarget);
        driveBackLeft.setTargetPosition(driveBackLeft.getCurrentPosition()+FrBlAxisTarget);
        driveBackRight.setTargetPosition(driveBackRight.getCurrentPosition()+FlBrAxisTarget);

        //set to run to position
        driveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set power
        setPower(FrBlPairPower, FlBrPairPower);

        //let RUN_TO_POSITION PID do its thing
        while (moving) {
            //are any of the motors busy?
            moving = driveFrontRight.isBusy() || driveFrontLeft.isBusy(); //save 6ms per loop by only reading from one motor of each diagonal pair
        }

        //stop and go back to normal
        setPower(0);
        driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    /**
     * rotates the robot by adjusting its current motor powers rather than directly setting powers, allows turning while strafing
     * @param signedPower determines the directions and power of rotation, larger values result in faster movement, and the sign (positive or negative) of this value determine direction
     */
    public void rotateByCorrection(double signedPower) {
        //largest current power + signedPower, or 1
        double denom = Math.max(
                Math.max(
                        Math.max( Math.abs(driveFrontRight.getPower()), Math.abs(driveFrontLeft.getPower())),
                        Math.max( Math.abs(driveBackRight.getPower()), Math.abs(driveBackLeft.getPower()))
                ) + Math.abs(signedPower),
                1.0
        );
        setPower(
                (driveFrontRight.getPower()-signedPower)/denom,
                (driveFrontLeft.getPower()+signedPower)/denom,
                (driveBackLeft.getPower()+signedPower)/denom,
                (driveBackRight.getPower()-signedPower)/denom
        );
    }
    /**
     * rotates the bot by directly setting powers
     * @param power the power to rotate at
     */
    public void rotate(double power) {
        setPower(
                -power,
                power,
                power,
                -power
        );
    }

    ////////////////////////////// Set Methods //////////////////////////////
    /**
     * set power to all drive motors individually
     * @param frontRightPower   power to assign to driveFrontRight
     * @param frontLeftPower    power to assign to driveFrontLeft
     * @param backLeftPower     power to assign to driveBackLeft
     * @param backRightPower    power to assign to driveBackRight
     */
    public void setPower(double frontRightPower, double frontLeftPower, double backLeftPower, double backRightPower){
        driveFrontRight.setPower(frontRightPower);
        driveFrontLeft.setPower(frontLeftPower);
        driveBackLeft.setPower(backLeftPower);
        driveBackRight.setPower(backRightPower);
    }
    /**
     * set all motors to the same power
     * @param power     power to assign to all drive motors
     */
    public void setPower(double power){
        setPower(power,power,power,power);
    }
    /**
     * set power to the diagonal pairs of drive motors
     * useful for strafing
     * @param FrBlPairPower     power to assign to the driveFrontRight-driveBackLeft diagonal pair of motors
     * @param FlBrPairPower     power to assign to the driveFrontLeft-driveBackRight diagonal pair of motors
     */
    public void setPower(double FrBlPairPower, double FlBrPairPower){
        setPower(FrBlPairPower,FlBrPairPower,FrBlPairPower,FlBrPairPower);
    }

    /**
     * runs the setVelocityPIDFCoefficients() method from DcMotorEx on all drive motors using passed parameters
     * @param pidfCoefficients coefficients to use
     */
    public void setVelocityPIDFCoefficients(PIDFCoefficients pidfCoefficients) {
        velocityPIDFCoefficients = new PIDFCoefficients(pidfCoefficients);
        double p,i,d,f;
        p=pidfCoefficients.p;
        i=pidfCoefficients.i;
        d=pidfCoefficients.d;
        f=pidfCoefficients.f;
        driveFrontRight.setVelocityPIDFCoefficients(p,i,d,f);
        driveFrontLeft.setVelocityPIDFCoefficients(p,i,d,f);
        driveBackLeft.setVelocityPIDFCoefficients(p,i,d,f);
        driveBackRight.setVelocityPIDFCoefficients(p,i,d,f);
    }
    /**
     * runs the setVelocityPIDFCoefficients() method from DcMotorEx on all drive motors using passed parameters
     * @param p Position coefficient
     * @param i Integral coefficient
     * @param d Derivative coefficient
     * @param f FeedForward coefficient
     */
    public void setVelocityPIDFCoefficients(double p,double i,double d,double f) {
        setVelocityPIDFCoefficients(new PIDFCoefficients(p,i,d,f));
    }
    /**
     * runs the setPositionPIDFCoefficients() method from DcMotorEx on all drive motors using passed parameters
     * @param p Position coefficient, (this controller only needs a Position term)
     */
    public void setPositionPIDFCoefficients(double p) {
        positionPIDFCoefficients.p = p;
        driveFrontRight.setPositionPIDFCoefficients(p);
        driveFrontLeft.setPositionPIDFCoefficients(p);
        driveBackLeft.setPositionPIDFCoefficients(p);
        driveBackRight.setPositionPIDFCoefficients(p);
    }
    /**
     * runs the setTargetPositionTolerance() method from DcMotorEx on all drive motors using passed parameters
     * @param tolerance tolerance (in encoder ticks)
     */
    public void setTargetPositionTolerance(int tolerance) {
        targetPositionTolerance = tolerance;
        driveFrontRight.setTargetPositionTolerance(tolerance);
        driveFrontLeft.setTargetPositionTolerance(tolerance);
        driveBackLeft.setTargetPositionTolerance(tolerance);
        driveBackRight.setTargetPositionTolerance(tolerance);
    }
    /**
     * runs the setMode() method from DcMotor on all drive motors using passed parameters
     * @param runMode RUNMODE to use
     */
    public void setMode(DcMotor.RunMode runMode) {
        driveFrontRight.setMode(runMode);
        driveFrontLeft.setMode(runMode);
        driveBackLeft.setMode(runMode);
        driveBackRight.setMode(runMode);
    }
    ////////////////////////////// Get Methods //////////////////////////////
    /**
     * @return a list containing all DcMotors in this component
     */
    @Override
    public List<HardwareDevice> getAll() {
        List<HardwareDevice> hardwareDeviceList = new LinkedList<>();
        hardwareDeviceList.add(driveFrontRight);
        hardwareDeviceList.add(driveFrontLeft);
        hardwareDeviceList.add(driveBackLeft);
        hardwareDeviceList.add(driveBackRight);
        return hardwareDeviceList;
    }


}
