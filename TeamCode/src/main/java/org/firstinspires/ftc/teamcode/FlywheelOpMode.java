package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class FlywheelOpMode extends LinearOpMode {
    //Variables

    private DcMotor flywheel;

    @Override
    public void runOpMode()
    {
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        //Wait for the start button to be pressed.
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        while (opModeIsActive())
        {
            controls();
        }
    }

    public void controls()
    {
        double gamepadtrigger = gamepad1.right_trigger;
        telemetry.addData("gamepad trigger", gamepadtrigger);

        flywheel.setPower(gamepadtrigger);
        telemetry.update();
    }
}