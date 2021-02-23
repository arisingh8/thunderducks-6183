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

    private DcMotor intakeMotor1, intakeMotor2;

    private DcMotor flywheel;
    private Servo servo;
    private EncoderReader flywheel_reader;

    private int shotCount = 1, powerShotCount = 1;
    private final double[] shotPower = {-0.98, -0.95, -0.95}, powerShotPower = {-0.98, -0.95, -0.95};
    private final double[] shotRevs = {5200, 5200, 5200}, powerShotRevs = {5000, 5000, 5000};

    RingStackPipeline pipeline = new RingStackPipeline();

    private double firingTime = 0.15;
    private double waitingTime = 0.35;
    private double servoStartPos = 0.15;
    private double servoFirePos = 0.4;

    private enum autofirePos {
        READY,
        FIRING,
        WAITING
    }
    private autofirePos servoState = autofirePos.READY;
    public boolean fired = false;
    public boolean normalfired = false;

    private enum wobbleGoalPos {
        READY,
        EXTENDED,
        DROPPED
    }
    private enum pickUpGoalPos {
        READY,
        EXTENDED,
        GRABBED
    }
    private wobbleGoalPos wobbleState = wobbleGoalPos.READY;
    private pickUpGoalPos pickUpState = pickUpGoalPos.READY;
    private boolean dropped = false, pickedUp = false;

    ElapsedTime eTime = new ElapsedTime();

    private OpenCvCamera camera;

    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        intakeMotor1 = hardwareMap.get(DcMotor.class, "intake1");
        intakeMotor2 = hardwareMap.get(DcMotor.class, "intake2");
        servo = hardwareMap.get(Servo.class, "servo");

        flywheel_reader = new EncoderReader(flywheel, 28, 0.1);
        servo.setPosition(servoStartPos);

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

    public void turnOnFlywheel() { flywheel.setPower(-1); }
    public void turnOffFlywheel() {
        flywheel.setPower(0);
    }

    public void fireHighGoalShot() {
        if (shotCount > 3) {
            shotCount = 1;
        }

        flywheel.setPower(shotPower[shotCount - 1]);
        switch (servoState) {
            case READY:
                if (-flywheel_reader.readCycle() > shotRevs[shotCount - 1]) {
                    shotCount += 1;

                    eTime.reset();
                    servoState = autofirePos.FIRING;
                }
                break;
            case FIRING:
                servo.setPosition(servoFirePos);
                if (eTime.time() > firingTime) {
                    servo.setPosition(servoStartPos);
                    servoState = autofirePos.WAITING;
                }
                break;
            case WAITING:
                if (eTime.time() > waitingTime) {
                    servoState = autofirePos.READY;
                    normalfired = true;
                }
                break;
        }
    }

    public void autoFireHighGoal(int shots) {
        for (int i = 0; i < shots; i++) {
            eTime.reset();
            while (!normalfired) {
                fireHighGoalShot();
            }
            normalfired = false;
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

    public void dropWobbleGoal() {
        switch (wobbleState) {
            case READY:
                setWobbleArmState(3);
                wobbleState = wobbleGoalPos.EXTENDED;
                break;
            case EXTENDED:
                if (Math.abs(arm.getCurrentPosition() - 2632) < 200) {
                    setWobbleClawState(false);
                    wobbleState = wobbleGoalPos.DROPPED;
                    eTime.reset();
                }
                break;
            case DROPPED:
                if (eTime.time() > 0.5) {
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

    public void pickUpWobbleGoal() {
        switch (pickUpState) {
            case READY:
                setWobbleArmState(3);
                if (arm.getCurrentPosition() > 600) {
                    setWobbleClawState(false);
                    pickUpState = pickUpGoalPos.EXTENDED;
                }
                break;
            case EXTENDED:
                if (Math.abs(arm.getCurrentPosition() - 2632) < 20) {
                    setWobbleClawState(true);
                    pickUpState = pickUpGoalPos.GRABBED;
                    eTime.reset();
                }
                break;
            case GRABBED:
                //if (eTime.time() > 1) {
                  //  setWobbleArmState(0);
                    pickedUp = true;
                    pickUpState = pickUpGoalPos.READY;
                //}
                break;
        }
    }

    public void autoDropWobbleGoal() {
        while (!dropped) {
            dropWobbleGoal();
        }
        dropped = false;
    }

    public void autoPickUpWobbleGoal() {
        while (!pickedUp) {
            pickUpWobbleGoal();
        }
        pickedUp = false;
    }

    public void turnOnIntake() {
        intakeMotor1.setPower(1);
        intakeMotor2.setPower(1);
    }

    public void turnOffIntake() {
        intakeMotor1.setPower(0);
        intakeMotor2.setPower(0);
    }
}
