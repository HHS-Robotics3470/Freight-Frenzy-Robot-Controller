package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/*
    NOTE -conventions for the measurement of distances and anlges-
    distances will be recorded and implemented in Standard units (meters), do not use the imperial system in your code
        this is because the metric system is easier to convert and work with than imperial units
    angles will be measured in radians
        this is due to how the Trig functions operate in java (they expect input, and return, values measured in radians)
        should be constricted to [pi, -pi) when possible
    angle 0:
        for the robot, heading of 0 means that it's facing to the right (POV: you're looking toward the far side of the field (far side meaning the side opposite to the side the robot starts on))
        for the horiz odometry, + change should mean turning CCW
                                - change should mean turning CW
    for the robots turret, negative headings mean port-side, positive are starboard-side
    Coords format: x,y,z
    * origin (0,0,0) : defined (for now) as where the robot starts, once we know what the event will be the origin will be set to the middle of the field
    *      field x bounds []
    *      field y bounds []
    * z = height from the foam grid
    * x+ =
    * x- =
    * y+ =
    * y- =
 */

/*
gamepad joystick signs
      y-neg
        ___
      /  |  \
x-neg|---+---|pos
      \__|__/
       y-pos
 */





/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for the robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces, or underscores, between words.
 *  more motors, sensors, servos, etc will be added as needed
 *
 *
 *
 *
 * the purpose of this class is to act as a centralized method for initializing the hardware on the
 * robot, so that the addition of hardware elements can be accounted for in one central area, reducing
 * the risk of error in the event the robot is updated
 *
 * this class should contain the initialization for the robot, as well as many of the methods the robot will use in its OpModes
 */

public class HardwareOutreach{
    ////////////////////////////// class variables //////////////////////////////
    /* --Public OpMode members.-- */
    //**Motors**//
    public DcMotor leftDrive,rightDrive; //drive motors
    public DcMotor flyWheel1,flyWheel2,turretRotator, turretElevator; //other motors //last two are hex motor & go bilda 53:1
    //**Servos**//
    public Servo turretLauncher;
    //**Sensors**//

    //**Other**//
    //IMU Sensor
    public BNO055IMU imu;
    private double globalAngle;
    private Orientation lastAngles = new Orientation();

    /* --local OpMode members.-- */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    //**variables for robot measurements**//
    /* some variables for different measurements of the robot */ //TODO: keep up to date
    public final double turretHeight = 0.15; //5 + (13/16) inches, from the floor to the launch platform at rest, up to date but not 100% accurate
    public static final long LAUNCHER_TIME_TO_ROTATE = 1400; //out of date, needs testing, this number represents how long it takes for the continuous servo to rotate one full rotation at full power

    //**hardware stats**//
    // stats for the TorqueNADO motors
    public final double NADO_COUNTS_PER_MOTOR_REV = 1440;
    public final double NADO_DRIVE_GEAR_REDUCTION = 32.0/24.0;  // This is < 1.0 if geared UP (to increase speed)
    public final double NADO_WHEEL_DIAMETER_METERS= 0.1016; //(4") For figuring circumference
    public final double NADO_COUNTS_PER_METER      = (NADO_COUNTS_PER_MOTOR_REV * NADO_DRIVE_GEAR_REDUCTION) /
            (NADO_WHEEL_DIAMETER_METERS * Math.PI);
    public final double NADO_METERS_PER_COUNT = 1.0 / NADO_COUNTS_PER_METER;

    // stats for the Rev Core Hex motor
    public final double CORE_HEX_COUNTS_PER_MOTOR_REV = 288;  // 4 * 72(gear ratio)
    public final double CORE_HEX_DRIVE_GEAR_REDUCTION = 1;    // This is < 1.0 if geared UP
    public final double CORE_HEX_RADIANS_PER_COUNTS   = (2 * Math.PI) / (CORE_HEX_COUNTS_PER_MOTOR_REV * CORE_HEX_DRIVE_GEAR_REDUCTION); //  radians per rotation / effective counts per rotation

    // stats for the goBilda series 5201 53:1 motor
    public final double GO_BILDA_COUNTS_PER_MOTOR_REV   = 1497.325;
    public final double GO_BILDA_DRIVE_GEAR_REDUCTION   = 1; // This is < 1.0 if geared UP
    public final double GO_BILDA_RADIANS_PER_COUNTS     = (2 * Math.PI) / (GO_BILDA_COUNTS_PER_MOTOR_REV * GO_BILDA_DRIVE_GEAR_REDUCTION); //  radians per rotation / effective counts per rotation


    ////////////////////////////// Constructors and init method //////////////////////////////
    /* --Constructors-- */
    public HardwareOutreach(){
    }

    /* --Initialize standard Hardware interfaces-- */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        /*initialize hardware components*/
        /* Motors */
        initMotors();

        /* Servos */
        initServos();

        /* Sensors */
        initSensors();

        /* Other */
        //IMU (Inertial Motion Unit)
        initIMU();
    }
    //motors
    private void initMotors() {
        initDriveMotors();
        initTurretMotors();
    }
    private void initDriveMotors() {
        // Define and initialize all Motors
        leftDrive   = hwMap.get(DcMotor.class, "leftDrive"); //___ hub, motor port ___,
        rightDrive  = hwMap.get(DcMotor.class, "rightDrive"); //___ hub, motor port ___,

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        // Set run modes
        //reset encoders
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //run using encoders
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  //torqueNADO motor
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //torqueNADO motor
        //run without encoders

        // Set Zero power behavior
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //assign motor directions
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
    }
    private void initTurretMotors() {
        // Define and initialize all Motors
        flyWheel1   = hwMap.get(DcMotor.class, "flywheelLeft"); //___ hub, motor port ___,
        flyWheel2   = hwMap.get(DcMotor.class, "flywheelRight"); //___ hub, motor port ___,
        turretRotator = hwMap.get(DcMotor.class, "turretRotate"); //___ hub, motor port ___,
        turretElevator = hwMap.get(DcMotor.class, "turretElevator"); //___ hub, motor port ___,

        // Set all motors to zero power
        flyWheel1.setPower(0);
        flyWheel2.setPower(0);
        turretRotator.setPower(0);
        turretElevator.setPower(0);


        // Set run modes
        //reset encoders
        turretRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //run using encoders
        turretRotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //core hex motor //will run using a target position
        turretElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);// GoBilda 5201 series 53:1 //will run using a target position
        //run without encoders
        flyWheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyWheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Set Zero power behavior
        flyWheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyWheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turretRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //assign motor directions
        flyWheel1.setDirection(DcMotor.Direction.FORWARD);
        flyWheel2.setDirection(DcMotor.Direction.REVERSE);
        turretRotator.setDirection(DcMotor.Direction.REVERSE);
        turretElevator.setDirection(DcMotor.Direction.FORWARD);
    }
    //servos
    private void initServos() {
        // Define and initialize ALL installed servos.
        turretLauncher = hwMap.get(Servo.class, "turretLaunchServo");
        // Set start positions for ALL installed servos
        //1 = firing position; 0.4 = reload
        turretLauncher.setPosition(1);

    }
    //sensors
    private void initSensors() {
        // Define and initialize all Sensors
    }
    //other
    private void initIMU() {
        //Initialize IMU hardware map value. PLEASE UPDATE THIS VALUE TO MATCH YOUR CONFIGURATION
        imu = hwMap.get(BNO055IMU.class, "imu");
        //Initialize IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }

    ////////////////////////////// Methods //////////////////////////////
    /**
     * runs a given motor (that has an encoder) to a given position, at a given power
     * @param motor the motor to move
     * @param targetPosition    the position to move to
     * @param power the power to move at (must be positive)
     */
    public void runMotorToPosition(DcMotor motor, int targetPosition, double power) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(targetPosition);
        motor.setPower(0);
        if (targetPosition > motor.getCurrentPosition()) motor.setPower(power);
        else if (targetPosition < motor.getCurrentPosition()) motor.setPower(-power);

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (motor.isBusy()) {} //let the motor run to that position

        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    ////////////////////////////// Get Methods //////////////////////////////
    /**
     * Gets the orientation of the robot using the REV IMU
     * @return the angle of the robot
     */
    public double getZAngle(){
        // TODO: This must be changed to match your configuration
        //                           | Z axis
        //                           |
        //     (Motor Port Side)     |   / X axis
        //                       ____|__/____
        //          Y axis     / *   | /    /|   (IO Side)
        //          _________ /______|/    //      I2C
        //                   /___________ //     Digital
        //                  |____________|/      Analog
        //
        //                 (Servo Port Side)
        //
        // The positive x axis points toward the USB port(s)
        //
        // Adjust the axis rotation rate as necessary
        // Rotate about the z axis is the default assuming your REV Hub/Control Hub is laying
        // flat on a surface
        return (-imu.getAngularOrientation().firstAngle);
    }
    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in radians
     */
    public double getGlobalAngle() {
        // TODO: This must be changed to match your configuration
        //                           | Z axis
        //                           |
        //     (Motor Port Side)     |   / X axis
        //                       ____|__/____
        //          Y axis     / *   | /    /|   (IO Side)
        //          _________ /______|/    //      I2C
        //                   /___________ //     Digital
        //                  |____________|/      Analog
        //
        //                 (Servo Port Side)
        //
        // The positive x axis points toward the USB port(s)
        //
        // Adjust the axis rotation rate as necessary
        // Rotate about the z axis is the default assuming your REV Hub/Control Hub is laying
        // flat on a surface
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -Math.PI)      deltaAngle+=2*Math.PI;
        else if (deltaAngle > Math.PI)  deltaAngle-=2*Math.PI;

        globalAngle += deltaAngle;
        lastAngles = angles;

        return globalAngle;
    }
    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        globalAngle = 0;
    }
}
