package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;

@TeleOp

public class TestOpMode extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor rfTest;
    private DcMotor rbTest;
    private DcMotor lfTest;
    private DcMotor lbTest;


    @Override
    public void runOpMode() {
        imu = hardwareMap.get(Gyroscope.class, "imu");
        rfTest = hardwareMap.get(DcMotor.class, "rfMotor");
        rbTest = hardwareMap.get(DcMotor.class, "rbMotor");
        lfTest = hardwareMap.get(DcMotor.class, "lfMotor");
        lbTest = hardwareMap.get(DcMotor.class, "lbMotor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}