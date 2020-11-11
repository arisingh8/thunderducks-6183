package org.firstinspires.ftc.teamcode.ChiefKeef.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ChiefKeef.EncoderReader;

public class Shooter {
    private DcMotor flywheel;
    private Servo servo;
    private Telemetry telemetry;

    private double g1rt;
    private boolean g1ab;

    private EncoderReader flywheel_reader;

    ElapsedTime eTime = new ElapsedTime();
    ElapsedTime restTime = new ElapsedTime();

    public void init(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        servo = hardwareMap.get(Servo.class, "servo");

        flywheel_reader = new EncoderReader(flywheel, 28, 0.1);
        servo.setPosition(0.15);

        eTime.reset();
        restTime.reset();
    }

    public void controls() {
        // EncoderRead();
        double rpm = flywheel_reader.readCycle();
        telemetry.addData("Shooter RPM", rpm);

        launchEm(rpm);
    }

    public void shoot(double g1rt, boolean g1ab, Telemetry telemetry) {
        this.g1rt = g1rt;
        this.g1ab = g1ab;
        this.telemetry = telemetry;

        controls();
    }

    public void launchEm(double rpm) {
        flywheel.setPower(g1rt);

        if (rpm > 5500 && eTime.time() > 0.125) {
            servo.setPosition(0.4);
            if (eTime.time() > 0.25) {
                servo.setPosition(0.15);
                eTime.reset();
            }
        }

        if (rpm < 5500) {
            servo.setPosition(0.15);
        }
    }
}