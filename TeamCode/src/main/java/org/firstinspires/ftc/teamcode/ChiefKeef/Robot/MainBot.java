package org.firstinspires.ftc.teamcode.ChiefKeef.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ChiefKeef.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.ChiefKeef.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.ChiefKeef.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.ChiefKeef.Subsystems.WobbleGoalArm;

@TeleOp(name = "MainTeleOp", group = "tele")
public class MainBot extends OpMode {
    Drivetrain drivetrain = new Drivetrain();
    Shooter shooter = new Shooter();
    Intake intake = new Intake();
    WobbleGoalArm wobbleGoalArm = new WobbleGoalArm();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multiTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

    @Override
    public void init() {
        drivetrain.init(hardwareMap);
        shooter.init(hardwareMap);
        intake.init(hardwareMap);
        wobbleGoalArm.init(hardwareMap);
    }

    @Override
    public void loop() {
        shooter.shoot(gamepad1.right_trigger, gamepad1.left_trigger, gamepad1.right_bumper, multiTelemetry);
        intake.intake(gamepad1.b, gamepad1.y, multiTelemetry);
        wobbleGoalArm.pickUp(gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.dpad_down, gamepad1.dpad_up, gamepad1.a, gamepad1.right_bumper, multiTelemetry);
        drivetrain.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.dpad_down, gamepad1.dpad_up, multiTelemetry);

        multiTelemetry.update();
    }
}
