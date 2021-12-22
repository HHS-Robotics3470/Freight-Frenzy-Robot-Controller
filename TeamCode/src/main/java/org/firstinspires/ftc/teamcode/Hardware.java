package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Components.CascadeOutputSystem;
import org.firstinspires.ftc.teamcode.Components.Component;
import org.firstinspires.ftc.teamcode.Components.IntakeSystem;
import org.firstinspires.ftc.teamcode.Components.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Utility.PIDController;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

/*
    NOTE -conventions for the measurement of distances and angles-
    distances will be recorded and implemented in Standard units (meters), do not use the imperial system in your code
        this is because the metric system is easier to convert and work with than imperial units
    angles will be measured in radians
        this is due to how the Trig functions operate in java (they expect input, and return, values measured in radians)
        should be constricted to [pi, -pi) when possible
    angle 0:
        for the robot, heading of 0 means that it's facing same diretion as  x+ axis
        for the horiz odometry, + change should mean turning CCW
                                - change should mean turning CW

    Coords format: x,y,z
    * origin (0,0,0) : defined (for now) as where the robot starts, once we know what the event will be the origin will be set to the middle of the field
    *      field x bounds []
    *      field y bounds []
    * z = height from the foam grid
    * x+ = blue alliance side
    * x- = red alliance side
    * y+ = warehouse side (and shared shipping hub)
    * y- = shipping unit side

    robot starts autonomous with back (side w/ webcam) facing barcodes

    front of robot is always the side with the main: intake, output, webcam (go in that order (i.e. if robot has no input, the front is the side w/ the output, if robot has no input or output, front is side with the main encoder)
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
 *
 * configuration naming conventions:
 * Note:  All names are lower case and some have single spaces, or underscores, between words.
 *  more motors, sensors, servos, etc will be added as needed
 * 
 * naming convention for all hardware elements (in config file)
 * "{location}_{purpose}_{type}_{(optional) number}"
 *  ex: front_right_drive_motor
 *      back_color_sensor_1
 *      cascade_lift_motor
 * 
 *
 * the purpose of this class is to act as a centralized method for initializing the hardware on the
 * robot, so that the addition of hardware elements can be accounted for in one central area, reducing
 * the risk of error in the event the robot is updated
 *
 * this class should contain the initialization for the robot, as well as many of the methods the robot will use in its OpModes
 * @author Anthony Rubick
 */
public class Hardware implements Component {
    ////////////////////////////// class variables //////////////////////////////
    //TODO: in the future, maybe isolate specific mechanisms into their own "hardware" classes
    // (like: drivetrain, cascade, frontCollector, etc.) that could include
    // more specific methods and variables without cluttering this class so much
    // they'd be in their own folder too

    //params for hardware elements, store positions and whatnot that equate to known output, IE servo position for a claw to open/close,
    //first layer: specific hardware element (ie "cascade lift", "front input servo", etc.)
    //      second layer, parameters (k,v) (ie. ("open", 0.6), etc.)
    //public Map<String, Map<String, Double>> params;

    /* --Public OpMode members.-- */
    //**Components**//
    public MecanumDriveTrain driveTrain = new MecanumDriveTrain();
    public IntakeSystem intakeSystem = new IntakeSystem();
    public CascadeOutputSystem cascadeOutputSystem = new CascadeOutputSystem();

    //**Motors**//
    public DcMotor turntableMotor; //other motors

    //**Servos**//

    //**Sensors**//

    //**Other**//
    //IMU
    public BNO055IMU imu;
    private double globalAngle;
    private Orientation lastAngles = new Orientation();
    //Webcam
    private WebcamName vuforiaWebcam;
    //expansion hubs
    List<LynxModule> allHubs;

    /* --local OpMode members.-- */
    HardwareMap hwMap           =  null;
    private ElapsedTime runtime  = new ElapsedTime();
    //PID controllers
    public PIDController FrBlStrafePIDController, FlBrStrafePIDController, rotatePIDController;
    private PIDFCoefficients pidfCoefficients;

    //**variables for robot measurements**//

    //**odometry direction multipliers**//

    //**hardware stats**//
    // stats for the TorqueNADO motors
    public final double NADO_COUNTS_PER_MOTOR_REV = 1440;
    public final double NADO_DRIVE_GEAR_REDUCTION = 1;  // This is < 1.0 if geared UP (to increase speed)
    public final double NADO_WHEEL_DIAMETER_METERS= 0.2032; //(8") For figuring circumference
    public final double NADO_COUNTS_PER_METER      = (NADO_COUNTS_PER_MOTOR_REV * NADO_DRIVE_GEAR_REDUCTION) /
            (NADO_WHEEL_DIAMETER_METERS * Math.PI);
    public final double NADO_METERS_PER_COUNT = 1.0 / NADO_COUNTS_PER_METER;

    /* --Constructors-- */
    public Hardware(){

    }

    /* --Initialize standard Hardware interfaces-- */
    @Override
    public void init(HardwareMap ahwMap) {
        runtime.reset();

        // Save reference to Hardware map
        hwMap = ahwMap;

        /*initialize hardware components*/
        /* Drive Train */
        driveTrain.init(hwMap);
        /* Intake */
        intakeSystem.init(hwMap);
        /* Output */
        cascadeOutputSystem.init(hwMap);

        /* Motors */
        initMotors();

        /* Servos */

        /* Sensors */
        initSensors();

        /* Other */
        //IMU (Inertial Motion Unit)
        initIMU();
        //webcam and vuforia
        initWebcamAndVuforia();
        //PID's
        initPIDs();
        //hubs
        allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) { //optimizes sensor reading by caching in bulk, effectively reading all sensors in the same time as one,
                                            //however you must clear the BulkCache once per control cycle, see teleOpProgramTemplate for how
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    /**
     * @return a list of all hardware devices included on the component
     */
    @Override
    public List<HardwareDevice> getAll() {
        return null;
    }

    //motors
    private void initMotors()
    {
        //drive motors

        //other motors
        turntableMotor = hwMap.get(DcMotor.class,  "front_left_turntable_motor"); //expansion hub port 2
        // Set power to zero
        turntableMotor.setPower(0);
        // Set run modes
        turntableMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Set zero power behavior
        turntableMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // assign motor directions
        turntableMotor.setDirection(DcMotor.Direction.FORWARD);
    }
    //sensors
    private void initSensors() {
        // Define and initialize all Sensors

    }
    //other
    //imu
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
    //webcam
    private void initWebcamAndVuforia() {
        //webcam

        //TODO: add vuforia configs adeel
    }
    /*
    calculated coefficients for a torquenado motor with no load.
    target and input values are the encoder count of the motor, it's a position PID, and while it may work for velocity, velocity was not what it was tuned for.
                            Kp      Ki      Kd
    PID                 |.004602|.095875| 0.000055
    PI                  |.003452|0.04315|
    P                   |.003835|       |
    PD                  |.006136|       | 0.000074
    some overshoot      |.002557|.053271| 0.000082
    no overshoot        |.001534|.031958| 0.000049
    pessen integration  |.005369|.139818| 0.000077
    */
    /**
     * init PIDs with default coefficients
     */
    private void initPIDs() {

        initPIDs(
                new PIDCoefficients(0.001534,0.031958,0.000049),//0.004602, 0.04315, 0.000055), //frbl strafe pid
                new PIDCoefficients(0.001534,0.031958,0.000049),//0.004602, 0.04315, 0.000055), //flbr strafe pid
                new PIDCoefficients(0.004602, 0.04315, 0.000055)  //rotation pid
        );
    }
    /**
     * init PIDs with custom coefficients
     * @param FrBlStrafe    coefficients for the frontRight-backLeft axis strafing PID
     * @param FlBrStrafe    coefficients for the frontLeft-backRight axis strafing PID
     * @param rotate        coefficients for the rotation PID
     */
    private void initPIDs(PIDCoefficients FrBlStrafe, PIDCoefficients FlBrStrafe, PIDCoefficients rotate) {
        //save coefficients
        //**PIDS**//
        //PID coefficients

        //initialize PID controllers
        FrBlStrafePIDController     = new PIDController(FrBlStrafe);
        FlBrStrafePIDController     = new PIDController(FlBrStrafe);
        rotatePIDController         = new PIDController(rotate);

        //configure PID controllers
        FrBlStrafePIDController.reset();
        FrBlStrafePIDController.setOutputRange(-1,1);   //(-1,1) because those are the range of powers for DcMotors
        FrBlStrafePIDController.setTolerance(1);        //percent
        //FrBlStrafePIDController.enable();

        FlBrStrafePIDController.reset();
        FlBrStrafePIDController.setOutputRange(-1,1);   //(-1,1) because those are the range of powers for DcMotors
        FlBrStrafePIDController.setTolerance(1);        //percent
        //FlBrStrafePIDController.enable();

        rotatePIDController.reset();
        rotatePIDController.setInputRange(-Math.PI, Math.PI);   // convention for angles
        rotatePIDController.setOutputRange(-1,1);               // (-1,1) because those are the range of powers for DcMotors
        rotatePIDController.setTolerance(1);                    // percent
        rotatePIDController.setContinuous(true);                // we want input angles to wrap, because angles wrap
        //rotatePIDController.enable();
    }

    ////////////////////////////// Methods //////////////////////////////

    ////////////////////////////// Set Methods //////////////////////////////

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
