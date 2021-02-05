package org.firstinspires.ftc.teamcode.ChiefKeef;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class AutoFunctions extends LinearOpMode {
    private DcMotor arm;
    private Servo claw;

    private DcMotor flywheel;
    private Servo servo;
    private EncoderReader flywheel_reader;
    public int powerShotCount = 1;

    private final double[] powerShotPower = {-0.92, -0.90, -0.90};
    private final double[] powerShotRevs = {4800, 4800, 4800};

    RingStackPipeline pipeline = new RingStackPipeline();

    private enum autofirePos {
        READY,
        FIRING,
        WAITING
    }
    private autofirePos servoState = autofirePos.READY;
    public boolean fired = false;

    private enum wobbleGoalPos {
        READY,
        EXTENDED,
        DROPPED
    }
    private wobbleGoalPos wobbleState = wobbleGoalPos.READY;
    public boolean dropped = false;

    ElapsedTime eTime = new ElapsedTime();

    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        servo = hardwareMap.get(Servo.class, "servo");

        flywheel_reader = new EncoderReader(flywheel, 28, 0.1);
        servo.setPosition(0.15);

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm = hardwareMap.get(DcMotor.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setTargetPosition(0);
        arm.setPower(1);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        claw.setPosition(0);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

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

        eTime.reset();

        telemetry.addLine("Robot initialized.");
        telemetry.update();
    }

    public int getRingStack() {
        while (!isStarted()) {
            if (pipeline.getRingStack() == 0) {
                telemetry.addData("Ring Stack", "Zero");
            } else if (pipeline.getRingStack() == 1) {
                telemetry.addData("Ring Stack", "One");
            } else if (pipeline.getRingStack() == 4) {
                telemetry.addData("Ring Stack", "Four");
            } else {
                telemetry.addData("Ring Stack", pipeline.getRingStack());
            }
            telemetry.update();
        }
        return pipeline.getRingStack();
    }

    public void hsvTelemetry() {
        telemetry.addData("HSV Color", pipeline.getHsvOutput());
    }

    public void firePowerShot() {
        if (powerShotCount > 3) {
            powerShotCount = 1;
        }

        flywheel.setPower(powerShotPower[powerShotCount - 1]);
        switch (servoState) {
            case READY:
                if (-flywheel_reader.readCycle() > powerShotRevs[powerShotCount - 1]) {
                    powerShotCount += 1;

                    eTime.reset();
                    servoState = autofirePos.FIRING;
                }
                break;
            case FIRING:
                servo.setPosition(0.5);
                if (eTime.time() > 0.5) {
                    servo.setPosition(0);
                    servoState = autofirePos.WAITING;
                }
                break;
            case WAITING:
                if (eTime.time() > 0.75) {
                    servoState = autofirePos.READY;
                    fired = true;
                }
                break;
        }
    }

    public void autoFirePowerShot() {
        eTime.reset();
        while (!fired) {
            firePowerShot();
        }
        fired = false;
        //flywheel.setPower(0);
    }

    public void turnOffFlywheel() {
        flywheel.setPower(0);
    }

    public void setWobbleArmState(int state) {
        if (state == 0) {
            arm.setTargetPosition(0);
        } else if (state == 1) {
            arm.setTargetPosition(1316);
        } else if (state == 2) {
            arm.setTargetPosition(1974);
        } else if (state == 3) {
            arm.setTargetPosition(2632);
        }
    }

    public void setWobbleClawState(boolean closed) {
        if (closed) {
            claw.setPosition(0);
        } else {
            claw.setPosition(0.25);
        }
    }

    public void dropWobbleGoal() {
        switch (wobbleState) {
            case READY:
                setWobbleArmState(3);
                wobbleState = wobbleGoalPos.EXTENDED;
                break;
            case EXTENDED:
                if (Math.abs(arm.getCurrentPosition() - 2632) < 20) {
                    setWobbleClawState(false);
                    wobbleState = wobbleGoalPos.DROPPED;
                    eTime.reset();
                }
                break;
            case DROPPED:
                if (eTime.time() > 1) {
                    setWobbleArmState(0);
                    if (arm.getCurrentPosition() < 600) {
                        setWobbleClawState(true);
                        dropped = true;
                        wobbleState = wobbleGoalPos.READY;
                    }
                }
                break;
        }
    }

    public void autoDropWobbleGoal() {
        while (!dropped) {
            dropWobbleGoal();
        }
        dropped = false;
    }
}
