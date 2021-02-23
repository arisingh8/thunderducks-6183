package org.firstinspires.ftc.teamcode.ChiefKeef;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class WobbleGoalArmOpMode extends LinearOpMode {
    private DcMotor arm;
    private Servo claw;

    @Override
    public void runOpMode() {
        arm = hardwareMap.get(DcMotor.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");

        waitForStart();

        while (opModeIsActive()) {
            controls();
        }
    }

    public void controls() {
        if (gamepad1.left_bumper) {
            arm.setPower(0.2);
        } else {
            arm.setPower(0);
        }
        if (gamepad1.left_trigger > 0.5) {
            arm.setPower(-0.2);
        } else {
            arm.setPower(0);
        }
        if (gamepad1.right_bumper) {
            claw.setPosition(0);
        }
        if (gamepad1.right_trigger>0.5) {
            claw.setPosition(0.5);
        }
        telemetry.addData("Claw position", claw.getPosition());
        telemetry.update();
    }
}
