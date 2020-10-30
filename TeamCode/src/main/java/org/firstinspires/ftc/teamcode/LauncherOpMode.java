package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class LauncherOpMode extends LinearOpMode {

    private DcMotor flywheel;
    private Servo servo;

    @Override
    public void runOpMode()
    {
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        servo = hardwareMap.get(Servo.class, "servo");

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
        telemetry.addData("gamepad button", gamepad1.a);

        flywheel.setPower(gamepadtrigger);

        if (gamepad1.a) {
            servo.setPosition(0.44);
        } else {
            servo.setPosition(0);
        }

        telemetry.update();
    }
}