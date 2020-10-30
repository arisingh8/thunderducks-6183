package org.firstinspires.ftc.teamcode.ChiefKeef.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ChiefKeef.Subsystems.Drivetrain;

public class MainBot extends OpMode {
    public Drivetrain drivetrain = new Drivetrain();

    @Override
    public void init() {
        drivetrain.init(hardwareMap);
    }

    @Override
    public void loop() {
        drivetrain.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_button, telemetry);
        telemetry.update();
    }
}
