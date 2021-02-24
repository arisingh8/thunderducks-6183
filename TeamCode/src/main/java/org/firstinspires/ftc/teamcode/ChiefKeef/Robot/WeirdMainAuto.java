package org.firstinspires.ftc.teamcode.ChiefKeef.Robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ChiefKeef.EncoderReader;
import org.firstinspires.ftc.teamcode.ChiefKeef.RingStackPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class WeirdMainAuto extends LinearOpMode {
    private DcMotor arm;
    private Servo claw;

    private DcMotor intakeMotor1, intakeMotor2;

    private DcMotor flywheel;
    private Servo servo;
    private EncoderReader flywheel_reader;

    private int shotCount = 1, powerShotCount = 1;
    private final double[] shotPower = {-0.98, -0.95, -0.95}, powerShotPower = {-0.98, -0.95, -0.95};
    private final double[] shotRevs = {5200, 5200, 5200}, powerShotRevs = {5000, 5000, 5000};

    private double firingTime = 0.15;
    private double waitingTime = 0.35;
    private double servoStartPos = 0.15;
    private double servoFirePos = 0.4;

    private enum autofirePos {
        READY,
        FIRING,
        WAITING,
        IDLE
    }
    private autofirePos servoState = autofirePos.IDLE;
    private int targetShots = 0;

    private enum wobbleGoalPos {
        READY,
        EXTENDED,
        DROPPED,
        IDLE
    }
    private int targetWobble = 0, oldWobble = 0;
    private boolean targetClaw = false, oldClaw = false;
    private enum trajectoryState {
        BEGINNING,
        SHOOT1,
        DROPOFF1,
        PICKUPRINGS,
        PICKUPRINGS2,
        PICKUPWOBBLE,
        DROPOFF2,
        MOVETOSHOOT2,
        SHOOT2,
        PARK,
        IDLE
    }
    private trajectoryState trajState = trajectoryState.IDLE;
    private wobbleGoalPos wobbleState = wobbleGoalPos.IDLE;
    private boolean dropped = false, pickedUp = false;

    ElapsedTime eTime = new ElapsedTime();

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

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory ringPickUp = null, ringPickUp2 = null, wobble2, wobblePickUp, shoot2 = null, end = null;
        Pose2d start = new Pose2d(-62, -5, Math.toRadians(0));

        drive.setPoseEstimate(start);

        Trajectory beginning = drive.trajectoryBuilder(start)
                .splineToConstantHeading(new Vector2d(-28, 8), 0)
                .addDisplacementMarker(() -> {
                    turnOnFlywheel();
                })
                .splineToConstantHeading(new Vector2d(-5.25, -6), 0)
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

        telemetry.addLine("Robot initialized.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        RingStackPipeline.ringstackTime.reset();
        ringstack = getRingStack();
        while (ringstack == -1) {
            ringstack = getRingStack();
        }

        if (ringstack == 1) {
            ringPickUp = drive.trajectoryBuilder(wobbleb.end())
                    .splineToConstantHeading(new Vector2d(-22.5, -12), Math.toRadians(270))
                    .build();
            ringPickUp2 = drive.trajectoryBuilder(ringPickUp.end())
                    .lineToConstantHeading(new Vector2d(-26.5, -28.5))
                    .build();
            wobblePickUp = drive.trajectoryBuilder(ringPickUp2.end())
                    .splineToConstantHeading(new Vector2d(-57, -24), Math.toRadians(270))
                    .build();
            wobble2 = drive.trajectoryBuilder(wobblePickUp.end())
                    .splineToConstantHeading(new Vector2d(21, -3), Math.toRadians(90))
                    .build();
            shoot2 = drive.trajectoryBuilder(wobble2.end())
                    .addDisplacementMarker(() -> {
                        turnOnFlywheel();
                    })
                    .splineToLinearHeading(new Pose2d(-5.25, -6, 0), 0)
                    .build();
        } else {
            if (ringstack == 0) {
                wobblePickUp = drive.trajectoryBuilder(wobblea.end())
                        .splineToConstantHeading(new Vector2d(-57, -24), Math.toRadians(270))
                        .build();
                wobble2 = drive.trajectoryBuilder(wobblePickUp.end())
                        .splineToConstantHeading(new Vector2d(21, -3), Math.toRadians(90))
                        .build();
            } else {
                wobblePickUp = drive.trajectoryBuilder(wobblec.end())
                        .splineToConstantHeading(new Vector2d(-57, -24), Math.toRadians(270))
                        .build();
                wobble2 = drive.trajectoryBuilder(wobblePickUp.end())
                        .splineToConstantHeading(new Vector2d(54.5, -23), Math.toRadians(90))
                        .build();
            }
            end = drive.trajectoryBuilder(wobble2.end())
                    .lineToConstantHeading(new Vector2d(5.5, 0))
                    .build();
        }

        trajState = trajectoryState.BEGINNING;
        drive.followTrajectoryAsync(beginning);

        while (opModeIsActive() && !isStopRequested()) {
            if (ringstack == 1) {
                switch (trajState) {
                    case BEGINNING:
                        // Check if the drive class isn't busy
                        // `isBusy() == true` while it's following the trajectory
                        // Once `isBusy() == false`, the trajectory follower signals that it is finished
                        // We move on to the next state
                        // Make sure we use the async follow function
                        if (!drive.isBusy()) {
                            trajState = trajectoryState.SHOOT1;
                            targetShots = 3;
                        }
                        break;
                    case SHOOT1:
                        if (servoState == autofirePos.IDLE) {
                            trajState = trajectoryState.DROPOFF1;
                            drive.followTrajectoryAsync(wobbleb);
                            targetWobble = 3;
                            targetClaw = false;
                        }
                        break;
                    case DROPOFF1:
                        // Check if the drive class is busy following the trajectory
                        // Move on to the next state, TURN_1, once finished
                        if (!drive.isBusy() && wobbleState == wobbleGoalPos.IDLE) {
                            trajState = trajectoryState.PICKUPRINGS;
                            drive.followTrajectoryAsync(ringPickUp);
                        }
                        break;
                    case PICKUPRINGS:
                        // Check if the drive class is busy turning
                        // If not, move onto the next state, TRAJECTORY_3, once finished
                        if (!drive.isBusy()) {
                            turnOnIntake();
                            trajState = trajectoryState.PICKUPRINGS2;
                            drive.followTrajectoryAsync(ringPickUp2);
                        }
                        break;
                    case PICKUPRINGS2:
                        // Check if the drive class is busy turning
                        // If not, move onto the next state, TRAJECTORY_3, once finished
                        if (!drive.isBusy()) {
                            turnOffIntake();
                            trajState = trajectoryState.PICKUPWOBBLE;
                            drive.followTrajectoryAsync(wobblePickUp);
                        }
                        break;
                    case PICKUPWOBBLE:
                        // Check if the drive class is busy following the trajectory
                        // If not, move onto the next state, WAIT_1
                        if (!drive.isBusy()) {
                            targetWobble = 3;
                            targetClaw = true;
                            trajState = trajectoryState.DROPOFF2;
                            drive.followTrajectoryAsync(wobble2);
                        }
                        break;
                    case DROPOFF2:
                        if (!drive.isBusy()) {
                            targetWobble = 3;
                            targetClaw = false;
                            trajState = trajectoryState.MOVETOSHOOT2;
                            drive.followTrajectoryAsync(shoot2);
                        }
                        break;
                    case MOVETOSHOOT2:
                        // Check if the drive class is busy turning
                        // If not, move onto the next state, IDLE
                        // We are done with the program
                        if (!drive.isBusy()) {
                            targetShots = 1;
                            trajState = trajectoryState.SHOOT2;
                        }
                        break;
                    case SHOOT2:
                        // Check if the drive class is busy turning
                        // If not, move onto the next state, IDLE
                        // We are done with the program
                        if (servoState == autofirePos.IDLE) {
                            trajState = trajectoryState.PARK;
                            drive.turnAsync(90);
                        }
                        break;
                    case PARK:
                        // Check if the drive class is busy turning
                        // If not, move onto the next state, IDLE
                        // We are done with the program
                        if (!drive.isBusy()) {
                            trajState = trajectoryState.IDLE;
                        }
                        break;
                    case IDLE:
                        // Do nothing in IDLE
                        // currentState does not change once in IDLE
                        // This concludes the autonomous program
                        setWobbleClawState(true);
                        setWobbleArmState(0);
                        break;
                }
            } else {
                switch (trajState) {
                    case BEGINNING:
                        // Check if the drive class isn't busy
                        // `isBusy() == true` while it's following the trajectory
                        // Once `isBusy() == false`, the trajectory follower signals that it is finished
                        // We move on to the next state
                        // Make sure we use the async follow function
                        if (!drive.isBusy()) {
                            trajState = trajectoryState.SHOOT1;
                            targetShots = 3;
                        }
                        break;
                    case SHOOT1:
                        if (servoState == autofirePos.IDLE) {
                            trajState = trajectoryState.DROPOFF1;
                            drive.followTrajectoryAsync(wobbleb);
                            targetWobble = 3;
                            targetClaw = false;
                        }
                        break;
                    case DROPOFF1:
                        // Check if the drive class is busy following the trajectory
                        // Move on to the next state, TURN_1, once finished
                        if (!drive.isBusy() && wobbleState == wobbleGoalPos.IDLE) {
                            trajState = trajectoryState.PICKUPWOBBLE;
                            drive.followTrajectoryAsync(wobblePickUp);
                        }
                        break;
                    case PICKUPWOBBLE:
                        // Check if the drive class is busy following the trajectory
                        // If not, move onto the next state, WAIT_1
                        if (!drive.isBusy()) {
                            targetClaw = true;
                            trajState = trajectoryState.DROPOFF2;
                            drive.followTrajectoryAsync(wobble2);
                        }
                        break;
                    case DROPOFF2:
                        if (!drive.isBusy()) {
                            targetClaw = false;
                            trajState = trajectoryState.PARK;
                            drive.followTrajectoryAsync(end);
                        }
                        break;
                    case PARK:
                        // Check if the drive class is busy turning
                        // If not, move onto the next state, IDLE
                        // We are done with the program
                        if (!drive.isBusy()) {
                            trajState = trajectoryState.IDLE;
                        }
                        break;
                    case IDLE:
                        // Do nothing in IDLE
                        // currentState does not change once in IDLE
                        // This concludes the autonomous program
                        setWobbleClawState(true);
                        setWobbleArmState(0);
                        break;
                }
            }
            drive.update();

            fireHighGoalShot();
            dropWobbleGoal();
        }
    }

    /*
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
     */

    public void turnOnFlywheel() { flywheel.setPower(-1); }

    public void fireHighGoalShot() {
        if (shotCount > 3) {
            shotCount = 1;
        }

        switch (servoState) {
            case IDLE:
                if (targetShots != 0) {
                    servoState = autofirePos.READY;
                }
                break;
            case READY:
                flywheel.setPower(shotPower[shotCount - 1]);
                if (-flywheel_reader.readCycle() > shotRevs[shotCount - 1]) {
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
                    shotCount += 1;
                    if (shotCount <= targetShots) {
                        servoState = autofirePos.READY;
                    } else {
                        flywheel.setPower(0);
                        targetShots = 0;
                        servoState = autofirePos.IDLE;
                    }
                }
                break;
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
            case IDLE:
                if (targetWobble != oldWobble || targetClaw != oldClaw) {
                    wobbleState = wobbleGoalPos.READY;
                }
                break;
            case READY:
                setWobbleArmState(targetWobble);
                wobbleState = wobbleGoalPos.EXTENDED;
                break;
            case EXTENDED:
                if (Math.abs(arm.getCurrentPosition() - 2632) < 200) {
                    setWobbleClawState(targetClaw);
                    wobbleState = wobbleGoalPos.DROPPED;
                    //eTime.reset();
                }
                break;
            case DROPPED:
                //if (eTime.time() > 0.5) {
                  //  setWobbleArmState(0);
                    //if (arm.getCurrentPosition() < 600) {
                      //  setWobbleClawState(true);
                        oldWobble = targetWobble;
                        oldClaw = targetClaw;
                        wobbleState = wobbleGoalPos.IDLE;
                    //}
                //}
                break;
        }
    }

    public void turnOnIntake() {
        intakeMotor1.setPower(1);
        intakeMotor2.setPower(1);
    }

    public void turnOffIntake() {
        intakeMotor1.setPower(0);
        intakeMotor2.setPower(0);
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
