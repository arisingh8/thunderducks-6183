package org.firstinspires.ftc.teamcode.ChiefKeef.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ChiefKeef.EncoderReader;
import org.firstinspires.ftc.teamcode.ChiefKeef.PoseStorage;

@Config
public class Shooter {
    private DcMotor flywheel;
    private Servo servo;
    private Telemetry telemetry;
    private EncoderReader flywheel_reader;

    private double g1rt, g1lt;
    private boolean g1rb;

    private int shotCount = 1, powerShotCount = 1;
    private boolean powerfiring = false, standardfiring = false, override = false;
    private final double[] shotPower = {-0.98, -0.95, -0.95}, powerShotPower = {-0.98, -0.95, -0.95};
    private final double[] shotRevs = {5400, 5400, 5400}, powerShotRevs = {5000, 5000, 5000};

    public ElapsedTime eTime = new ElapsedTime();

    public static double multiplier = 1.7;
    private double targetRPM = multiplier * 120 * (PoseStorage.distanceToGoal+1.806) / Math.PI*0.04128;

    public enum firePos {
        READY,
        FIRING,
        WAITING
    }
    firePos servoState = firePos.READY;

    public void init(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        servo = hardwareMap.get(Servo.class, "servo");

        flywheel_reader = new EncoderReader(flywheel, 28, 0.1);
        servo.setPosition(0.15);

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        eTime.reset();
    }

    public void controls() {
        double rpm = flywheel_reader.readCycle();
        targetRPM = multiplier * 120 * (PoseStorage.distanceToGoal+1.806) / Math.PI*0.04128;
        telemetry.addData("Shooter RPM", rpm);

        launchEm(rpm);
    }

    public void shoot(double g1rt, double g1lt, boolean g1rb, Telemetry telemetry) {
        this.g1rt = g1rt;
        this.g1lt = g1lt;
        this.g1rb = g1rb;
        this.telemetry = telemetry;

        controls();
    }

    public void launchEm(double rpm) {
        if (shotCount > 3) {
            shotCount = 1;
        }
        if (powerShotCount > 3) {
            powerShotCount = 1;
        }

        switch (servoState) {
            case READY:
                if (g1rt > 0.5) {
                    powerfiring = false;
                    standardfiring = true;
                    if (g1rb) {
                        override = true;
                        flywheel.setPower(shotPower[shotCount-1]);
                    } else {
                        flywheel.setPower(targetRPM/6000 + 0.05);
                    }
                } else if (g1lt > 0.5) {
                    standardfiring = false;
                    powerfiring = true;
                    flywheel.setPower(powerShotPower[powerShotCount-1]);
                } else {
                    flywheel.setPower(0);
                }

                if (override) {
                    if (standardfiring && -rpm > shotRevs[shotCount-1]) {
                        powerShotCount = 1;
                        shotCount += 1;

                        eTime.reset();
                        servoState = firePos.FIRING;
                    }
                } else {
                    if (standardfiring && -rpm > targetRPM) {
                        powerShotCount = 1;
                        shotCount += 1;

                        eTime.reset();
                        servoState = firePos.FIRING;
                    }
                }
                if (powerfiring && -rpm > powerShotRevs[powerShotCount-1]) {
                    shotCount = 1;
                    powerShotCount += 1;

                    eTime.reset();
                    servoState = firePos.FIRING;
                }
                break;
            case FIRING:
                servo.setPosition(0.5);
                if (eTime.time() > 0.5) {
                    servo.setPosition(0);
                    servoState = firePos.WAITING;
                }
                break;
            case WAITING:
                if (eTime.time() > 0.75) {
                    if (standardfiring) {
                        servoState = firePos.READY;
                        override = false;
                        standardfiring = false;
                    } else if (powerfiring) {
                        if (g1lt < 0.2) {
                            servoState = firePos.READY;
                            override = false;
                            powerfiring = false;
                        }
                    }
                }
                break;
        }
    }
}