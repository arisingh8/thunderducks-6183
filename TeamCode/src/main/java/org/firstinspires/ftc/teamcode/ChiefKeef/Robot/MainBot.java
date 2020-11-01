package org.firstinspires.ftc.teamcode.ChiefKeef.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ChiefKeef.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.ChiefKeef.Subsystems.Shooter;

public class MainBot extends OpMode {
    Drivetrain drivetrain = new Drivetrain();
    // Shooter shooter = new Shooter();

    @Override
    public void init() {
        drivetrain.init(hardwareMap);
    }

    @Override
    public void loop() {
        drivetrain.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_button, telemetry);
        // shooter.shoot(gamepad1.right_trigger, gamepad1.a, telemetry);

        telemetry.update();
    }
}
