package org.firstinspires.ftc.teamcode.EasyOpenCv.CamOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class OpenCVBlobOpmode extends OpMode {

    //Resolution
    static final int STREAM_WIDTH = 1280; // modify for your camera
    static final int STREAM_HEIGHT = 720; // modify for your camera
    OpenCvWebcam webcam;
    OpenCVBlobPipeline pipeline;

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}

class OpenCVBlobPipeline extends OpenCvPipeline {

    @Override
    public Mat processFrame(Mat input) {
        System.out.println("processing requested");
        return null;
    }
}