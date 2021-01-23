package org.firstinspires.ftc.teamcode.ChiefKeef.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ChiefKeef.EncoderReader;
import org.firstinspires.ftc.teamcode.ChiefKeef.ShooterTargetingPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Shooter {
    private DcMotor flywheel;
    private Servo servo;
    private Telemetry telemetry;

    private double g1rt, g1lt;

    private double tRevs;
    private boolean firstShot;

    private EncoderReader flywheel_reader;

    OpenCvCamera webcam;
    ShooterTargetingPipeline pipeline;
    public boolean onTarget;

    public ElapsedTime eTime = new ElapsedTime();

    public enum firePos {
        READY,
        FIRSTSHOT,
        POWERSHOT,
        POWERWAIT,
        FIRING,
        WAITING
    }
    firePos servoState = firePos.READY;

    public void init(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        servo = hardwareMap.get(Servo.class, "servo");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new ShooterTargetingPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
        });
        flywheel_reader = new EncoderReader(flywheel, 28, 0.1);
        servo.setPosition(0.15);

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        eTime.reset();
    }

    public void controls() {
        // EncoderRead();
        double rpm = flywheel_reader.readCycle();
        telemetry.addData("Shooter RPM", rpm);

        launchEm(rpm);
    }

    public void shoot(double g1rt, double g1lt, double tRevs, Telemetry telemetry) {
        this.g1rt = g1rt;
        this.g1lt = g1lt;
        this.telemetry = telemetry;
        this.tRevs = tRevs;

        controls();
    }

    public void launchEm(double rpm) {
        double firstShotPower = -0.98;
        double normalPower = -0.95;
        double powershotPower = -0.9;
        int powershotRevs = 5000;
        switch (servoState) {
            case POWERWAIT:
                if (eTime.time() > 0.75 && g1lt < 0.5) {
                    servoState = firePos.READY;
                }
                break;
            case POWERSHOT:
                if (eTime.time() > 0.5) {
                    servo.setPosition(0);
                    servoState = firePos.WAITING;
                }
                break;
            case READY:
                if (g1rt > 0.5) {
                    if (firstShot) {
                        flywheel.setPower(firstShotPower);
                    } else {
                        flywheel.setPower(normalPower);
                    }
                } else if (g1lt > 0.5) {
                    flywheel.setPower(powershotPower);
                } else {
                    flywheel.setPower(0);
                    firstShot = true;
                }
                if (-rpm > tRevs && g1rt > 0.5) {
                    eTime.reset();
                    servo.setPosition(0.5);
                    if (firstShot) {
                        servoState = firePos.FIRSTSHOT;
                    } else {
                        servoState = firePos.FIRING;
                    }
                }
                if (-rpm > powershotRevs && g1lt > 0.5) {
                    eTime.reset();
                    servo.setPosition(0.5);
                    servoState = firePos.POWERSHOT;
                }
                break;
            case FIRSTSHOT:
                if (eTime.time() > 0.1) {
                    servoState = firePos.FIRING;
                    eTime.reset();
                }
                break;
            case FIRING:
                if (eTime.time() > 0.5) {
                    servo.setPosition(0);
                    firstShot = false;
                    flywheel.setPower(normalPower);
                    servoState = firePos.WAITING;
                }
                break;
            case WAITING:
                if (eTime.time() > 0.75) {
                    servoState = firePos.READY;
                }
                break;
        }
    }
}