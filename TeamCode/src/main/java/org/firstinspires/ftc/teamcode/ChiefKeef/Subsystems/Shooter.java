package org.firstinspires.ftc.teamcode.ChiefKeef.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ChiefKeef.EncoderReader;
import org.firstinspires.ftc.teamcode.ChiefKeef.ShooterTargetingPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class Shooter {
    private DcMotor flywheel;
    private Servo servo;
    private Telemetry telemetry;

    private double g1rt;
    private boolean g1ab;

    private double tRevs;

    private EncoderReader flywheel_reader;

    OpenCvCamera webcam;
    ShooterTargetingPipeline pipeline;
    boolean onTarget;

    ElapsedTime eTime = new ElapsedTime();
    ElapsedTime restTime = new ElapsedTime();

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
        flywheel_reader = new EncoderReader(flywheel, 28, 0.05);
        servo.setPosition(0.15);

        eTime.reset();
        restTime.reset();
    }

    public void controls() {
        // EncoderRead();
        double rpm = flywheel_reader.readCycle();
        telemetry.addData("Shooter RPM", rpm);
        if (pipeline.getAvgResults() == 112) {
            onTarget = true;
        } else {
            onTarget = false;
        }
        telemetry.addData("On target?", onTarget);
        telemetry.addData("avg1", pipeline.getAvgResults());

        launchEm(rpm);
    }

    public void shoot(double g1rt, boolean g1ab, double tRevs, Telemetry telemetry) {
        this.g1rt = g1rt;
        this.g1ab = g1ab;
        this.telemetry = telemetry;
        this.tRevs = tRevs;

        controls();
    }

    public void launchEm(double rpm) {
        double newPower = 0;
        if(g1rt > 0.5) {
            newPower = -(tRevs + rpm);
        }
        flywheel.setPower(newPower);

        /*
        if (eTime.time() > 0.125) {
            servo.setPosition(0.4);
            if (eTime.time() > 0.25) {
                servo.setPosition(0.15);
                eTime.reset();
            }
        }

        if (rpm > -tRevs) {
            servo.setPosition(0.15);
        }
        */


/*
        flywheel.setPower(-g1rt);

        if (rpm < -tRevs) {
            double newPower = (-(tRevs+0) - rpm - 50)/50;

            if(-rpm < tRevs){newPower = 1;}
            else{newPower = 0.8;}
            if (newPower > 1) { newPower = 1; }
            flywheel.setPower(-g1rt+newPower);
            if (eTime.time() > 0.125) {
                servo.setPosition(0.4);
                if (eTime.time() > 0.25) {
                    servo.setPosition(0.15);
                    eTime.reset();
                }
            }
 */




    }
}