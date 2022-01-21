package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import org.firstinspires.ftc.teamcode.Components.CascadeOutputSystem;
import org.firstinspires.ftc.teamcode.Components.Component;
import org.firstinspires.ftc.teamcode.Components.IntakeSystem;
import org.firstinspires.ftc.teamcode.Components.MecanumDriveTrain;

import java.util.List;
/*
    NOTE -conventions for the measurement of distances and angles-
    distances will be recorded and implemented in Standard units (meters), do not use the imperial system in your code
        this is because the metric system is easier to convert and work with than imperial units
    angles will be measured in radians
        this is due to how the Trig functions operate in java (they expect input, and return, values measured in radians)
        should be constricted to [pi, -pi) when possible
    angle 0:
        for the robot,
            relative to field, heading of 0 means that it's facing same diretion as  x+ axis
            relative to robot, heading/angle of 0 faces toward the starboard side of the robot
        for the horiz odometry,
            + change should mean turning CCW
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
    //TODO: update if this changes, it's true as of Jan/16/2021
    in this case, front is the side with the input
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
 * @author Adeel Ahmad - Vuforia
 */
public class Hardware implements Component {
    ////////////////////////////// class variables //////////////////////////////
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
    //Webcam vuforia and tensorflow
    private WebcamName vuforiaWebcam;
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    //expansion hubs
    List<LynxModule> allHubs;

    //**hardware stats**//
    // stats for the TorqueNADO motors
    public final double NADO_COUNTS_PER_MOTOR_REV = 1440;
    public final double NADO_DRIVE_GEAR_REDUCTION = 1;  // This is < 1.0 if geared UP (to increase speed)
    public final double NADO_WHEEL_DIAMETER_METERS= 0.2032; //(8") For figuring circumference
    public final double NADO_COUNTS_PER_METER      = (NADO_COUNTS_PER_MOTOR_REV * NADO_DRIVE_GEAR_REDUCTION) /
            (NADO_WHEEL_DIAMETER_METERS * Math.PI);
    public final double NADO_METERS_PER_COUNT = 1.0 / NADO_COUNTS_PER_METER;

    /* --local OpMode members.-- */
    HardwareMap hwMap           =  null;
    private ElapsedTime runtime  = new ElapsedTime();
    private static final String VUFORIA_KEY = "AW/D8Tv/////AAABmaz//hdMn0Nmm2YoSrW8emZqrSTAb26m/pJRCgy4GeNX6aO6frTzk1FQ/y8IC0mbDWke8NXa87KACa/HR1kVRqaamTM60GJcobyaZaK1k0NAkVZ94iJY/RlWsIzESF3hql3ADHV9oHUuSvZWAVkF8f01xr4bzFtLrXgORIxOFKsT4TWSfHIr1pZel50uC0psgWIWpcDFGY3wTHlcfahX93OY8rqz98vwZC6b2u0MiikDwFjzKD2zxtSvQkYyIogyccKwZrC4z432K1GwxSvUanLJVsNypOcDqVrXWJdHKSmJSuQ8Zrl5SDvPXFewBpBYUTacsrdIx6bUykW+hSTcMxFzMo8MHjrv+FYgtJwaVsFT";
    private boolean vuforiaEnabled = false;
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
        //done only in opmodes that need it
        //hubs
        allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) { //optimizes sensor reading by caching in bulk, effectively reading all sensors in the same time as one,
                                            //however you must clear the BulkCache once per control cycle, see teleOpProgramTemplate for how
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    //motors
    private void initMotors()
    {
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

    //vuforia stuff
    public void initVuforiaAndTfod(HardwareMap ahwMap) {
        //if vuforia already enabled, disable then re-enable

        if (hwMap == null) {
            hwMap=ahwMap;// Save reference to Hardware map
        }

        //webcam
        vuforiaWebcam = hwMap.get(WebcamName.class, "vuforia_webcam");

        //vuforia and tensorflow
        initVuforia();
        initTfod();
        vuforiaEnabled = true;
    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = vuforiaWebcam;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    private void initTfod() {

        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.4f; //TODO: update and tune this value
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        // getUpdatedRecognitions() will return null if no new information is available since
        // the last time that call was made.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);


        tfod.activate();

        // The TensorFlow software will scale the input images from the camera to a lower resolution.
        // This can result in lower detection accuracy at longer distances (> 55cm or 22").
        // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
        // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
        // should be set to the value of the images used to create the TensorFlow Object Detection model
        // (typically 16/9).
        tfod.setZoom(1, 16.0/9.0);


    }

    ////////////////////////////// Methods //////////////////////////////
    /**
     * @return a list of all hardware devices included on the component
     */
    @Override
    public List<HardwareDevice> getAll() {
        return null;
    }
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
