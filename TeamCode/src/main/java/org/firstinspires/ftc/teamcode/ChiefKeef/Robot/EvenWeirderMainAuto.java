package org.firstinspires.ftc.teamcode.ChiefKeef.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ChiefKeef.AutoFunctions;
import org.firstinspires.ftc.teamcode.ChiefKeef.RingStackPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class EvenWeirderMainAuto extends LinearOpMode {
    public int ringstack;
    private OpenCvCamera camera;
    private RingStackPipeline pipeline = new RingStackPipeline();

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.setPipeline(pipeline);
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
        });

        waitForStart();
        RingStackPipeline.ringstackTime.reset();
        ringstack = getRingStack();
        while (ringstack == -1) {
            ringstack = getRingStack();
        }
        telemetry.addData("Ring Stack", getRingStack());
        telemetry.update();
    }

    private int getRingStack() {
        int ringStackSize = pipeline.getRingStack();
        if (ringStackSize == 0) {
            telemetry.addData("Ring Stack", "Zero");
        } else if (ringStackSize == 1) {
            telemetry.addData("Ring Stack", "One");
        } else if (ringStackSize == 4) {
            telemetry.addData("Ring Stack", "Four");
        }
        telemetry.update();
        return ringStackSize;
    }
}