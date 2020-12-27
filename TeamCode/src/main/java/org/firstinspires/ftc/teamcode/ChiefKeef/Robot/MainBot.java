package org.firstinspires.ftc.teamcode.ChiefKeef.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ChiefKeef.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.ChiefKeef.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.ChiefKeef.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.ChiefKeef.Subsystems.WobbleGoalArm;

public class MainBot extends OpMode {
    Drivetrain drivetrain = new Drivetrain();
    Shooter shooter = new Shooter();
    Intake intake = new Intake();
    WobbleGoalArm wobbleGoalArm = new WobbleGoalArm();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    boolean oldg1y = false;
    boolean run = false;

    @Override
    public void init() {
        drivetrain.init(hardwareMap);
        shooter.init(hardwareMap);
        intake.init(hardwareMap);
        wobbleGoalArm.init(hardwareMap);
    }

    @Override
    public void loop() {
        shooter.shoot(gamepad1.right_trigger, gamepad1.a, 5400, dashboardTelemetry);
        intake.intake(gamepad1.b, dashboardTelemetry);
        wobbleGoalArm.pickUp(gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.dpad_down, gamepad1.dpad_up, dashboardTelemetry);

        if (gamepad1.y && !oldg1y) {
            run = !run;
        }
        oldg1y = gamepad1.y;

        if (run) {
            if (!shooter.isOnTarget()) {
                drivetrain.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_bumper, true, dashboardTelemetry);
            }
        } else {
            drivetrain.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_bumper, false, dashboardTelemetry);
        }

        dashboardTelemetry.update();
    }
}
