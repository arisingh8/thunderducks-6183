package org.firstinspires.ftc.teamcode.ChiefKeef.Robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ChiefKeef.AutoFunctions;
import org.firstinspires.ftc.teamcode.ChiefKeef.RingStackPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class MainAuto extends AutoFunctions {
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
        super.runOpMode();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory end, wobblePickUp, wobble2, ringPickUp, ringPickUp2;
        Pose2d start = new Pose2d(-62, -5, Math.toRadians(0));

        drive.setPoseEstimate(start);

        Trajectory beginning = drive.trajectoryBuilder(start)
                .splineToConstantHeading(new Vector2d(-28, 8), 0)
                .addDisplacementMarker(() -> {
                    turnOnFlywheel();
                })
                .splineToConstantHeading(new Vector2d(-5.25, -6), 0)
                .addDisplacementMarker(() -> {
                    autoFireHighGoal(3);
                    turnOffFlywheel();
                })
                .build();

        Trajectory wobblea = drive.trajectoryBuilder(beginning.end())
                .splineToLinearHeading(new Pose2d(5.5, -22.5,  Math.toRadians(90)), Math.toRadians(270))
                .build();

        Trajectory wobbleb = drive.trajectoryBuilder(beginning.end())
                .splineToLinearHeading(new Pose2d(29, -3, Math.toRadians(90)), Math.toRadians(270))
                .build();

        Trajectory wobblec = drive.trajectoryBuilder(beginning.end())
                .splineToLinearHeading(new Pose2d(54.5, -23, Math.toRadians(90)), Math.toRadians(270))
                .build();

        ringPickUp = drive.trajectoryBuilder(wobbleb.end())
                .splineToConstantHeading(new Vector2d(-22.5, -28.5), Math.toRadians(180))
                .build();

        ringPickUp2 = drive.trajectoryBuilder(ringPickUp.end())
                .lineToConstantHeading(new Vector2d(-26.5, -28.5))
                .build();

        waitForStart();
        RingStackPipeline.ringstackTime.reset();
        ringstack = getRingStack();
        while (ringstack == -1) {
            ringstack = getRingStack();
        }

        if (ringstack == 0) {
            wobblePickUp = drive.trajectoryBuilder(wobblea.end())
                    .splineToSplineHeading(new Pose2d(-48, -12, Math.toRadians(90)), Math.toRadians(270))
                    //.splineTo(new Vector2d(-48, -4), Math.toRadians(90))
                    .build();
            wobble2 = drive.trajectoryBuilder(wobblePickUp.end())
                    .splineToLinearHeading(new Pose2d(5.5, -22.5,  Math.toRadians(90)), Math.toRadians(270))
                    .build();
            ringPickUp = drive.trajectoryBuilder(wobblea.end())
                    .splineToConstantHeading(new Vector2d(-22.5, -28.5), Math.toRadians(180))
                    .addDisplacementMarker(() -> {
                        turnOnIntake();
                    })
                    .splineToConstantHeading(new Vector2d(-22.5, -36.5), Math.toRadians(180))
                    .addDisplacementMarker(() -> {
                        turnOffIntake();
                    })
                    .build();
        } else if (ringstack == 1) {
            wobblePickUp = drive.trajectoryBuilder(ringPickUp2.end())
                    .splineToConstantHeading(new Vector2d(-57, -24), Math.toRadians(270))
                    .build();
            wobble2 = drive.trajectoryBuilder(wobblePickUp.end())
                    .splineToConstantHeading(new Vector2d(21, -3), Math.toRadians(90))
                    .build();
            ringPickUp = drive.trajectoryBuilder(wobbleb.end())
                    .splineToConstantHeading(new Vector2d(-22.5, -12), Math.toRadians(270))
                    .build();
        } else {
            ringPickUp = drive.trajectoryBuilder(wobblec.end())
                    .splineToConstantHeading(new Vector2d(-26.5, -22.5), Math.toRadians(270))
                    .build();
            wobblePickUp = drive.trajectoryBuilder(ringPickUp2.end())
                    .splineToConstantHeading(new Vector2d(-57, -20), Math.toRadians(270))
                    .build();
            wobble2 = drive.trajectoryBuilder(wobblePickUp.end())
                    .splineToConstantHeading(new Vector2d(54.5, -23), Math.toRadians(90))
                    .build();
        }

        Trajectory wobblePickUp2 = drive.trajectoryBuilder(wobblePickUp.end())
                .lineToConstantHeading(new Vector2d(-57, -26))
                .build();

        ringPickUp2 = drive.trajectoryBuilder(ringPickUp.end())
                .lineToConstantHeading(new Vector2d(-26.5, -28.5))
                .build();

        end = drive.trajectoryBuilder(wobble2.end())
                .lineToConstantHeading(new Vector2d(5.5, 0))
                .build();

        drive.followTrajectory(beginning);

        if (ringstack == 0) {
            System.out.println("vargas: ringstack is 0");
            drive.followTrajectory(wobblea);
        } else if (ringstack == 1) {
            System.out.println("vargas: ringstack is 1");
            drive.followTrajectory(wobbleb);
        } else if (ringstack == 4) {
            System.out.println("vargas: ringstack is 4");
            drive.followTrajectory(wobblec);
        }

        autoDropWobbleGoal();
        System.out.println("vargas: wobble is dropped");

        drive.followTrajectory(ringPickUp);

        turnOnIntake();
        drive.followTrajectory(ringPickUp2);
        sleep(400);

        drive.followTrajectory(wobblePickUp);
        System.out.println("vargas: in pickup position");

        turnOffIntake();
        autoPickUpWobbleGoal();

        drive.followTrajectory(wobblePickUp2);
        System.out.println("vargas: 2nd wobble picked up");

        drive.followTrajectory(wobble2);
        System.out.println("vargas: in position for 2nd wobble drop");

        autoDropWobbleGoal();
        System.out.println("vargas: 2nd wobble dropped");

        drive.followTrajectory(end);
        System.out.println("vargas: opmode is done");
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