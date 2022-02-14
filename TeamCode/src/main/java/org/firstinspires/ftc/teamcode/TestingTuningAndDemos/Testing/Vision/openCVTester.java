package org.firstinspires.ftc.teamcode.TestingTuningAndDemos.Testing.Vision;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Components.vision.TestingPipeline;
import org.firstinspires.ftc.teamcode.Hardware;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/**
 * Template for Autonomous routines
 * @author Anthony Rubick
 */
@Autonomous(name="openCV tester", group="Testing" )
//@Disabled //this line disables the autonomous from appearing on the driver station, remove it for your code
public class openCVTester  extends LinearOpMode{
    /*declare OpMode members, initialize some classes*/
    Hardware robot          = new Hardware();
    ElapsedTime runtime     = new ElapsedTime();
    OpenCvCamera webcam;
    TestingPipeline pipeline;

    //this is the control loop, basically the equivalent of a main function almost
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode()
    {
        ////////////before driver presses play////////////
        //Variables

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        initOpencvTestPipeline();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        ////////////after driver presses play////////////

        //autonomous routine goes here step by step
        while (opModeIsActive()) {
            int level = pipeline.getAnalysis();
            Scalar centerAvg = pipeline.getCenterAvg();
            Scalar rightAvg = pipeline.getRightAvg();
            telemetry.addData("Position: ", level);
            telemetry.addLine(String.format("Center Averages: \n\tY: %d\tCr: %d\tCb: %d",
                    (int) centerAvg.val[0],
                    (int) centerAvg.val[1],
                    (int) centerAvg.val[2]
            ));
            telemetry.addLine(String.format("Right Averages: \n\tY: %d\tCr: %d\tCb: %d",
                    (int) rightAvg.val[0],
                    (int) rightAvg.val[1],
                    (int) rightAvg.val[2]
            ));
            telemetry.update();
        }

        ////////////after driver presses stop////////////
    }

    ////////////other methods and whatnot below here////////////
    private void initOpencvTestPipeline() {
        //open CV innit stuff
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "vuforia_webcam"), cameraMonitorViewId);
        pipeline = new TestingPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(
                        TestingPipeline.resWidth,
                        TestingPipeline.resHeight,
                        OpenCvCameraRotation.UPRIGHT
                ); //resolution and orientation of camera
            }

            @Override
            public void onError(int errorCode) { }
        });
    }
}
