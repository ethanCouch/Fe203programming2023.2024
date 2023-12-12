package org.firstinspires.ftc.teamcode.EasyOpenCv.CamOpModes;


import static org.opencv.features2d.Features2d.DrawMatchesFlags_DRAW_RICH_KEYPOINTS;
import static org.opencv.features2d.Features2d.drawKeypoints;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.opencv.core.Core;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Scalar;
import org.opencv.features2d.SimpleBlobDetector;
import org.opencv.core.Mat;
import org.opencv.features2d.SimpleBlobDetector_Params;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

// Useful documentation: https://docs.opencv.org/3.4/javadoc/org/opencv/features2d/SimpleBlobDetector.html#create()

// stack overflow: https://stackoverflow.com/questions/41116049/how-can-i-use-blob-detector#:~:text=To%20detect%20blobs%20with%20OpenCV%20you%20need%20to%3A,of%20type%20KeyPoint%20s%203%20Call%20SimpleBlobDetector%3A%3Adetect%20%28%29

/* object dimentions:
          ______
 2-4 in  |      |
         |      |
         |______|
 2-4 in

*/
@Autonomous(name="SimpleOpenCVBlobOpMode")
public class OpenCVBlobOpmode extends OpMode {



    //Resolution
    static final int STREAM_WIDTH = 1280; // modify for your camera
    static final int STREAM_HEIGHT = 720; // modify for your camera
    OpenCvWebcam webcam; 
    SimpleOpenCVBlobPipeline pipeline;

    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1"); // put your camera's name here
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new SimpleOpenCVBlobPipeline();
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


// Show blobs
        telemetry.addData("keypoints:", pipeline.getBlobAnalysis());
        // Display the image with keypoints
        Core.imshow("Keypoints", im);
        Core.waitKey(0);

    }
}

class SimpleOpenCVBlobPipeline extends OpenCvPipeline {
    Mat BlobImg;
    MatOfKeyPoint keypoints;
    Mat im_with_keypoints;
    SimpleBlobDetector_Params params;
    @Override
    public Mat processFrame(Mat input) {

        try {
// Read image
            BlobImg = Imgcodecs.imread("blob.jpg", Imgcodecs.IMREAD_GRAYSCALE);

            Imgproc.cvtColor(input, BlobImg, Imgproc.COLOR_RGB2GRAY);

//      Set min/max thresholds:
            params.set_minThreshold(50);
            params.set_maxThreshold(100);

//      Filter by min/max area:
            params.set_minArea(2);
//        params.set_maxArea(4);

// Create a detector with the parameters
// OLD: detector = cv2.SimpleBlobDetector(params)
            SimpleBlobDetector detector = SimpleBlobDetector.create(params);

            keypoints = detector.detect(BlobImg);

            SimpleBlobDetector.create(params).detect(BlobImg, keypoints);

//      Draw detected blobs as red circles.
//      DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
            drawKeypoints(BlobImg, keypoints, im_with_keypoints, new Scalar(0, 0, 225), DrawMatchesFlags_DRAW_RICH_KEYPOINTS);

            System.out.println("processing requested");



        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        return input;
    }

    public Mat getBlobAnalysis() {return im_with_keypoints;}
}