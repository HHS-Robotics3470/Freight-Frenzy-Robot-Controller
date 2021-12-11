package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.os.Handler;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue;
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

    Coords format: x,y,z
    * origin (0,0,0) : defined (for now) as where the robot starts, once we know what the event will be the origin will be set to the middle of the field
    *      field x bounds []
    *      field y bounds []
    * z = height from the foam grid
    * x+ =
    * x- =
    * y+ =
    * y- =

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
 */
public class Hardware {
    ////////////////////////////// class variables //////////////////////////////
    /* --Public OpMode members.-- */
    //**Motors**//
    public DcMotor driveFrontRight,driveFrontLeft,driveBackLeft,driveBackRight; //drive motors
    public DcMotor cascadeLiftMotor,turntableMotor; //other motors

    //**Servos**//
    public Servo cascadeOutputServo, cascadeFlipperServo, frontInputFlipperServo, frontInputServo;

    //**Sensors**//

    //**Other**//
    //IMU
    public BNO055IMU imu;
    private double globalAngle;
    private Orientation lastAngles = new Orientation();
    //Webcam
    private WebcamName vuforiaWebcam;

    /* --local OpMode members.-- */
    HardwareMap hwMap           =  null;
    private ElapsedTime runtime  = new ElapsedTime();
    //**PIDS**//
    //PID coefficients
    private PIDCoefficients FrBlStrafePIDCoeffecients, FlBrStrafePIDCoeffecients, rotatePIDCoeffecients;
    //PID controllers
    public PIDController FrBlStrafePIDController, FlBrStrafePIDController, rotatePIDController;

    //**variables for robot measurements**//

    //**odometry direction multipliers**//

    //**hardware stats**//
    // stats for the TorqueNADO motors
    public final double NADO_COUNTS_PER_MOTOR_REV = 1440;
    public final double NADO_DRIVE_GEAR_REDUCTION = 1;  // This is < 1.0 if geared UP (to increase speed)
    public final double NADO_WHEEL_DIAMETER_METERS= 0.1016; //(4") For figuring circumference
    public final double NADO_COUNTS_PER_METER      = (NADO_COUNTS_PER_MOTOR_REV * NADO_DRIVE_GEAR_REDUCTION) /
            (NADO_WHEEL_DIAMETER_METERS * Math.PI);
    public final double NADO_METERS_PER_COUNT = 1.0 / NADO_COUNTS_PER_METER;

    ////////////////////////////// Constructors and init method //////////////////////////////
    /* --Constructors-- */
    public Hardware(){

    }

    /* --Initialize standard Hardware interfaces-- */
    public void init(HardwareMap ahwMap) {
        runtime.reset();

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
        //webcam and vuforia
        initWebcamAndVuforia();
        //PID's
        initPIDs();
    }
    //motors
    private void initMotors()
    {
        //drive motors
        initDriveMotors();

        //other motors
        initOtherMotors();
    }
    private void initDriveMotors() {
        // Define and initialize all Motors
        driveFrontLeft  = hwMap.get(DcMotor.class, "front_left_drive");  //main hub port 0
        driveFrontRight = hwMap.get(DcMotor.class, "front_right_drive"); //main hub port 1
        driveBackRight  = hwMap.get(DcMotor.class, "back_right_drive");  //main hub port 2
        driveBackLeft   = hwMap.get(DcMotor.class, "back_left_drive");   //main hub port 3

        // Set all motors to zero power
        driveFrontRight.setPower(0);
        driveFrontLeft.setPower(0);
        driveBackLeft.setPower(0);
        driveBackRight.setPower(0);

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
    }
    private void initOtherMotors()
    {
        //define and initialize
        cascadeLiftMotor = hwMap.get(DcMotor.class, "cascade_lift_motor");        //expansion hub port 1 //max extension = 2150 counts
        turntableMotor = hwMap.get(DcMotor.class,  "front_left_turntable_motor"); //expansion hub port 2

        // Set power to zero
        cascadeLiftMotor.setPower(0);
        turntableMotor.setPower(0);

        // Set run modes
        cascadeLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cascadeLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turntableMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set zero power behavior
        cascadeLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turntableMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // assign motor directions
        cascadeLiftMotor.setDirection(DcMotor.Direction.FORWARD);
        turntableMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    //servos
    private void initServos() {
        // Define and initialize ALL installed servos.
        cascadeOutputServo = hwMap.get(Servo.class, "cascade_output_servo"); //closed 1, open to drop 0.75, open to receive 0.5
        cascadeFlipperServo = hwMap.get(Servo.class, "cascade_output_flipper_servo"); // retracted 0.125, extended flat 0.8, extended up 0.6, extended down 0.95
        frontInputFlipperServo = hwMap.get(Servo.class, "front_input_flipper_servo"); //down 0, up 0.7
        frontInputServo = hwMap.get(Servo.class, "front_input_servo");//full open 0.3; half open 0.55; closed 0.72

        // Set start positions for ALL installed servos
        cascadeOutputServo.setPosition(1);
        cascadeFlipperServo.setPosition(0.125);
        frontInputFlipperServo.setPosition(0);
        frontInputServo.setPosition(0.3);


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
        vuforiaWebcam = hwMap.get(WebcamName.class, "vuforia_webcam");

        //TODO: add vuforia configs adeel
    }
    //PIDS
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
                new PIDCoefficients(0.001534/3,0.031958/3,0.000049/3),//0.004602, 0.04315, 0.000055), //frbl strafe pid
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
    public void initPIDs(PIDCoefficients FrBlStrafe, PIDCoefficients FlBrStrafe, PIDCoefficients rotate) {
        //save coefficients
        FrBlStrafePIDCoeffecients   = FrBlStrafe;
        FlBrStrafePIDCoeffecients   = FlBrStrafe;
        rotatePIDCoeffecients       = rotate;

        //initialize PID controllers
        FrBlStrafePIDController     = new PIDController(FrBlStrafePIDCoeffecients);
        FlBrStrafePIDController     = new PIDController(FlBrStrafePIDCoeffecients);
        rotatePIDController         = new PIDController(rotatePIDCoeffecients);

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
    /**
     * runs a given motor (that has an encoder) to a given position, at a given power
     * @param motor the motor to move
     * @param targetPosition    the position to move to
     * @param power the power to move at (must be positive)
     */
    public void runMotorToPosition(DcMotor motor, int targetPosition, double power) {
        power = Math.abs(power);
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

    /**
     * given a power and direction, causes the robot to strafe continuously in that given direction at the given power
     * @param power power factor
     * @param angle direction to strafe relative to robot, measure in radians, angle of 0 == starboard
     */
    public void strafeDirection(double power, double angle) {
        // calculate the power that needs to be assigned to each diagonal pair of motors
        double FrBlPairPower = Math.sin(angle - (Math.PI/4)) * power;
        double FlBrPairPower = Math.sin(angle + (Math.PI/4)) * power;

        setDrivetrainPowerMecanum(FrBlPairPower, FlBrPairPower);
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
        //initial heading and encoder counts
        //double initialHeading = getGlobalAngle();
        int initialFrBlAxisEncoderCount = (driveFrontRight.getCurrentPosition() + driveBackLeft.getCurrentPosition())/2;
        int initialFlBrAxisEncoderCount = (driveFrontLeft.getCurrentPosition() + driveBackRight.getCurrentPosition())/2;

        //calculate desired power for each diagonal motor pair
        double FrBlPairPower = Math.sin(angle - (Math.PI/4));
        double FlBrPairPower = Math.sin(angle + (Math.PI/4));

        //find the desired target for each strafe PID
        int FrBlAxisTarget = (int) (targetDistance * (FrBlPairPower)); //in meters
        int FlBrAxisTarget = (int) (targetDistance * (FlBrPairPower)); //in meters
        // convert to encoder counts
        FrBlAxisTarget *= NADO_COUNTS_PER_METER;
        FlBrAxisTarget *= NADO_COUNTS_PER_METER;

        //set up the PIDs
        power = Math.abs(power);
        //FrBl PID
        FrBlStrafePIDController.reset();
        FrBlStrafePIDController.setSetpoint(FrBlAxisTarget);
        FrBlStrafePIDController.setOutputRange(-power*FrBlPairPower, power*FrBlPairPower);
        FrBlStrafePIDController.enable(runtime.seconds());
        //FlBr PID
        FlBrStrafePIDController.reset();
        FlBrStrafePIDController.setSetpoint(FlBrAxisTarget);
        FlBrStrafePIDController.setOutputRange(-power*FlBrPairPower, power*FlBrPairPower);
        FlBrStrafePIDController.enable(runtime.seconds());
        //rotation PID (will be used to maintain constant heading)
        //rotatePIDController.reset();
        //rotatePIDController.setSetpoint(initialHeading);
        //rotatePIDController.setOutputRange(-power, power);
        //rotatePIDController.enable(runtime.seconds());


        while (!FrBlStrafePIDController.onTarget() || !FlBrStrafePIDController.onTarget()) {
            //calculate distance travelled by each diagonal pair
            int FrBlAxisEncoderCount = (driveFrontRight.getCurrentPosition() + driveBackLeft.getCurrentPosition())/2;
            int FlBrAxisEncoderCount = (driveFrontLeft.getCurrentPosition() + driveBackRight.getCurrentPosition())/2;

            //get correction values
            FrBlPairPower = FrBlStrafePIDController.performPID(FrBlAxisEncoderCount - initialFrBlAxisEncoderCount,runtime.seconds());
            FlBrPairPower = FlBrStrafePIDController.performPID(FlBrAxisEncoderCount - initialFlBrAxisEncoderCount,runtime.seconds());
            //double rotationCorrection = rotatePIDController.performPID(getGlobalAngle(),runtime.seconds());

            //assign drive motor powers
            setDrivetrainPower(
                    FrBlPairPower,// - rotationCorrection,
                    FlBrPairPower,// + rotationCorrection,
                    FrBlPairPower,// + rotationCorrection,
                    FlBrPairPower// - rotationCorrection
            );
        }

    }

    /**
     * DOES NOT WORK AS EXPECTED
     * rotates the robot by adjusting its current motor powers rather than directly setting powers, allows turning while strafing
     * @param signedPower determines the directions and power of rotation, larger values result in faster movement, and the sign (positive or negative) of this value determine direction
     */
    public void rotateByCorrection(double signedPower) {
        setDrivetrainPower(
                driveFrontRight.getPower()-signedPower,
                driveFrontLeft.getPower()+signedPower,
                driveBackLeft.getPower()+signedPower,
                driveBackRight.getPower()-signedPower
        );
    }
    /**
     * rotates the bot by directly setting powers
     * @param power the power to rotate at
     */
    public void rotate(double power) {
        setDrivetrainPower(
                -power,
                power,
                power,
                -power
        );
    }

    /**
     * given an array of rectangular coordinates (x,y), returns array of equivalent polar coordinates (r, theta)
     * @param rectangularCoords array of length 2 containing rectangular (x,y) coordinates
     * @return array of length 2 containing equivalent polar coordinates (or null values if passed array is too big / too small)
     */
    public static double[] convertRectangularToPolar(double[] rectangularCoords) {
        //error prevention
        if (rectangularCoords.length != 2) return new double[2];
        else {
            //calculations
            double x = rectangularCoords[0];
            double y = rectangularCoords[1];
            double radius   = Math.sqrt((x * x) + (y * y));    //radius, distance formula
            double theta    = Math.atan2(y, x);                //theta, arctangent(y/x)

            return new double[]{radius,theta};
        }
    }

    /**
     * given rectangular coordinates (x,y), returns array of equivalent polar coordinates (r, theta)
     * @param x x-coordinate
     * @param y y-coordinate
     * @return array of length 2 containing equivalent polar coordinates
     */

    public static double[] convertRectangularToPolar(double x, double y) {
        return convertRectangularToPolar(new double[]{x, y});
    }

    /**
     * given an array of polar coordinates (r, theta), returns array of equivalent rectangular coordinates (x,y)
     * @param polarCoords array of length 2 containing polar (r,theta) coordinates
     * @return array of length 2 containing equivalent rectangular coordinates (or null values if passed array is too big / too small)
     */
    public static double[] convertPolarToRectangular(double[] polarCoords) {
        //error prevention
        if (polarCoords.length != 2) return new double[2];
        else {
            //calculations
            double radius   = polarCoords[0];
            double theta    = polarCoords[1];
            double x        = Math.cos(theta) * radius;
            double y        = Math.sin(theta) * radius;

            return new double[]{x,y};
        }
    }
    /**
     * given polar coordinates (r, theta), returns array of equivalent rectangular coordinates (x,y)
     * @param radius radius, distance
     * @param theta  theta, angle
     * @return array of length 2 containing equivalent rectangular coordinates
     */
    public static double[] convertPolarToRectangular(double radius, double theta) {
        return convertPolarToRectangular(new double[]{radius, theta});
    }

    ////////////////////////////// Set Methods //////////////////////////////
    /**
     * set power to all drive motors individually
     * @param frontRightPower   power to assign to driveFrontRight
     * @param frontLeftPower    power to assign to driveFrontLeft
     * @param backLeftPower     power to assign to driveBackLeft
     * @param backRightPower    power to assign to driveBackRight
     */
    public void setDrivetrainPower(double frontRightPower, double frontLeftPower, double backLeftPower, double backRightPower){
        driveFrontRight.setPower(frontRightPower);
        driveFrontLeft.setPower(frontLeftPower);
        driveBackLeft.setPower(backLeftPower);
        driveBackRight.setPower(backRightPower);
    }
    /**
     * set all motors to the same power
     * @param power     power to assign to all drive motors
     */
    public void setDrivetrainPower(double power){
        setDrivetrainPower(power,power,power,power);
    }
    /**
     * set power to the left and right motors
     * useful for traditional tank-like driving, and turning
     * @param rightPower    power to assign to the right drive motors
     * @param leftPower     power to assign to the left drive motors
     */
    public void setDrivetrainPowerTraditional(double rightPower, double leftPower){
        setDrivetrainPower(rightPower, leftPower, leftPower, rightPower);
    }
    /**
     * set power to the diagonal pairs of drive motors
     * useful for strafing with mecanum drivetrains
     * @param FrBlPairPower     power to assign to the driveFrontRight-driveBackLeft diagonal pair of motors
     * @param FlBrPairPower     power to assign to the driveFrontLeft-driveBackRight diagonal pair of motors
     */
    public void setDrivetrainPowerMecanum(double FrBlPairPower, double FlBrPairPower){
        setDrivetrainPower(FrBlPairPower,FlBrPairPower,FrBlPairPower,FlBrPairPower);
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