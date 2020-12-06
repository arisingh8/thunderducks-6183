package org.firstinspires.ftc.teamcode.ChiefKeef.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ChiefKeef.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.ChiefKeef.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.ChiefKeef.Subsystems.Shooter;

public class MainBot extends OpMode {
    Drivetrain drivetrain = new Drivetrain();
    Shooter shooter = new Shooter();
    Intake intake = new Intake();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void init() {
        drivetrain.init(hardwareMap);
        shooter.init(hardwareMap);
        intake.init(hardwareMap);
    }

    @Override
    public void loop() {
        drivetrain.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_button, dashboardTelemetry);
        shooter.shoot(gamepad1.right_trigger, gamepad1.a, 5400, dashboardTelemetry);
        intake.intake(gamepad1.b, dashboardTelemetry);

        dashboardTelemetry.update();
    }
}
