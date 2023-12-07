package org.firstinspires.ftc.teamcode.EasyOpenCv.CamOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous(name="SimpleOpenCVOpMode")
public class SimpleOpenCVColorOpMode extends OpMode {
    //Resolution
    static final int STREAM_WIDTH = 1280; // modify for your camera
    static final int STREAM_HEIGHT = 720; // modify for your camera
    OpenCvWebcam webcam;
    SimpleOpenCVColorPipeline pipeline;

    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1"); // put your camera's name here
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new SimpleOpenCVColorPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Failed","L");
                telemetry.update();
            }
        });
    }
    @Override
    public void loop() {

//      Create booleans Red and Blue

        Boolean Blue = false;

        Boolean Red = false;

//      Define YCrCbAnalysis so I don't have to keep typing "pipeline.get" :)
//      note: if this becomes an issue for memory/bandwidth replace this:
        double YAnalysis = pipeline.getYAnalysis();
        double CrAnalysis = pipeline.getCrAnalysis();
        double CbAnalysis = pipeline.getCbAnalysis();

//      If red-differance > 130 and blue-differance < 130 Blue = true
        if(CrAnalysis < 130)
        {
            if(CbAnalysis > 130)
            {
                Blue = true;
            }
        }

//      If blue-difference > 130 and red-difference < 130: Red = true
        if(CbAnalysis < 130)
        {
            if(CrAnalysis > 130)
            {
                Red = true;
            }
        }

//      Print YCrCb analysis to driver station
        telemetry.addData("Image Y Analysis:",YAnalysis);
        telemetry.addData("Image Cr Analysis:",CrAnalysis);
        telemetry.addData("Image Cb Analysis:",CbAnalysis);

//      Print true if  blue-difference > 130 and red-difference < 130
        telemetry.addData("Detected Blue?",Blue);

//      Print true if  red-difference  > 130 and blue-difference < 130
        telemetry.addData("Detected Red?",Red);

        telemetry.update();

    }


}

class SimpleOpenCVColorPipeline extends OpenCvPipeline {

    Mat YCrCb = new Mat();
    Mat Y = new Mat();
    Mat Cr = new Mat();
    Mat Cb = new Mat();
    int avgY;
    int avgCr;
    int avgCb;

    /*This function takes the RGB frame, converts to YCrCb,
    and extracts the Y channel to the 'Y' variable*/
    void inputToYCrCb(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        ArrayList<Mat> yCrCbChannels = new ArrayList<Mat>(3);
        Core.split(YCrCb, yCrCbChannels);
        Y = yCrCbChannels.get(0);
        Cr = yCrCbChannels.get(1);
        Cb = yCrCbChannels.get(2);}

    @Override
    public void init(Mat firstFrame) {
        inputToYCrCb(firstFrame);}

    @Override
    public Mat processFrame(Mat input) {
        inputToYCrCb(input);
        System.out.println("processing requested");

//      Average Y, Cr, and Cb values
        avgY = (int) Core.mean(Y).val[0];
        avgCr = (int) Core.mean(Cr).val[0];
        avgCb = (int) Core.mean(Cb).val[0];

        YCrCb.release(); // don't leak memory!
        Y.release(); // don't leak memory!
        Cr.release(); // don't leak memory!
        Cb.release(); // don't leak memory!

        return input;
    }


//  Return average Y, Cr, and Cb
    public int getYAnalysis() {
        return avgY;
    }
    public int getCrAnalysis() {
        return avgCr;
    }
    public int getCbAnalysis() {return avgCb;}

}