package org.firstinspires.ftc.teamcode.ChiefKeef;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class AutoFunctions extends LinearOpMode {
    private DcMotor arm;
    private Servo claw;

    private DcMotor flywheel;
    private Servo servo;
    private EncoderReader flywheel_reader;
    public int powerShotCount = 1;

    private final double[] powerShotPower = {-0.98, -0.95, -0.95};
    private final double[] powerShotRevs = {5000, 5000, 5000};

    private enum autofirePos {
        READY,
        FIRING,
        WAITING
    }
    private autofirePos servoState = autofirePos.READY;
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

        eTime.reset();

        telemetry.addLine("Robot initialized.");
        telemetry.update();
    }

    public void firePowerShot() {
        flywheel.setPower(-0.98);
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
                }
                break;
        }
    }

    public void fireAutoPowerShot() {
        boolean fired = false;
        flywheel.setPower(-0.98);

        autofirePos servoState = autofirePos.READY;
        eTime.reset();

        while (fired = false) {
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
                        fired = true;
                        flywheel.setPower(0);
                        servoState = autofirePos.READY;
                    }
                    break;
            }
        }
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
}
