package org.firstinspires.ftc.teamcode.ChiefKeef;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class WobbleGoalArmOpMode extends LinearOpMode {
    private DcMotor arm;

    @Override
    public void runOpMode() {
        arm = hardwareMap.get(DcMotor.class, "arm");

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
    }
}
