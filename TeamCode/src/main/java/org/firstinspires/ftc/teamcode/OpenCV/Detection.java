package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class Detection extends OpMode {

    OpenCvWebcam webcam1;

    @Override
    public void init() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam1.setPipeline(new detectionPipeline());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                webcam1.startStreaming(960,720, OpenCvCameraRotation.UPRIGHT);
            }
            public void onError(int errorCode) {
            }
        });

    }

    @Override
    public void loop() {

    }

    class detectionPipeline extends OpenCvPipeline{
        Mat YCbCr = new Mat();
        Mat aof;
        double aofavgfin;
        Mat output = new Mat();
        Scalar rectColor = new Scalar(0,0,0);

        public Mat processFrame(Mat input){

            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_BGR2YCrCb);
            telemetry.addLine("pipeline running");

            Rect aofRect = new Rect(320,145,640,290);

            input.copyTo(output);
            Imgproc.rectangle(output, aofRect, rectColor, 2);

            Core.extractChannel(aof, aof, 2);

            aof = YCbCr.submat(aofRect);

            Scalar aofavg = Core.mean(aof);

            return(output);
        }
    }
}