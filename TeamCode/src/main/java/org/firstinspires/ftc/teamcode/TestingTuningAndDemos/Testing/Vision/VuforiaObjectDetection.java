package org.firstinspires.ftc.teamcode.TestingTuningAndDemos.Testing.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.R;


/**
 * @author Adeel Ahmad
 */

@Autonomous(name = "Vuforia Object Detection", group = "AI Testing")
//@Disabled //this line disables the autonomous from appearing on the driver station, remove it for your code
public class VuforiaObjectDetection extends LinearOpMode{

    /*declare OpMode members, initialize some classes*/
    Hardware robot          = new Hardware();
    ElapsedTime runtime     = new ElapsedTime();

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
      "Ball",
      "Cube",
      "Duck",
      "Marker"
    };

    private static final String VUFORIA_KEY =
            "AW/D8Tv/////AAABmaz//hdMn0Nmm2YoSrW8emZqrSTAb26m/pJRCgy4GeNX6aO6frTzk1FQ/y8IC0mbDWke8NXa87KACa/HR1kVRqaamTM60GJcobyaZaK1k0NAkVZ94iJY/RlWsIzESF3hql3ADHV9oHUuSvZWAVkF8f01xr4bzFtLrXgORIxOFKsT4TWSfHIr1pZel50uC0psgWIWpcDFGY3wTHlcfahX93OY8rqz98vwZC6b2u0MiikDwFjzKD2zxtSvQkYyIogyccKwZrC4z432K1GwxSvUanLJVsNypOcDqVrXWJdHKSmJSuQ8Zrl5SDvPXFewBpBYUTacsrdIx6bUykW+hSTcMxFzMo8MHjrv+FYgtJwaVsFT";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        robot.init(hardwareMap);
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.2, 16.0/9.0);
        }

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            int level = -1;
            double val1 = 245; //mm relative to camera
            double val2 = 468; //mm relative to camera
            //val1 = 397.5;
            //val2 = 457;
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        if (recognition.getLabel().equals("Marker")) {
                            robot.driveTrain.strafeToDistance(0.4,0,0.2);
                            List<Recognition> updatedRecognitions2 = tfod.getUpdatedRecognitions();
                            for (Recognition recognition2: updatedRecognitions2) {
                                if (recognition2.getLabel().equals("Marker")) {
                                    level = 2;
                                    break;
                                } else {
                                    level = 0;
                                }
                            }
                            telemetry.addData("level: ", level);
                            //break;
                            robot.driveTrain.strafeToDistance(0.4,Math.PI,0.2);
                            break;
                        }
                        else {
                            level = 1;
                        }
                        i++;
                    }
                    telemetry.update();
                }
            }
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        // TODO: the R.id.cameraMonitorViewId shows what the camera sees on the driver station, remove it or comment out when not needed bc it drains battery fast
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "vuforia_webcam");
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        parameters.fillCameraMonitorViewParent = true;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
       tfodParameters.minResultConfidence = 0.4f;//0.55f;
       tfodParameters.isModelTensorFlow2 = true;
       tfodParameters.inputSize = 320;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
