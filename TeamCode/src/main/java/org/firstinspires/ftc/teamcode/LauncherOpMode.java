package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ChiefKeef.EncoderReader;

@TeleOp(name="Launcher Test", group = "Test Op Mode")
@Disabled
public class LauncherOpMode extends LinearOpMode {

    private DcMotor flywheel;
    private Servo servo;

    private boolean triggered;

    private EncoderReader flywheel_reader;

    ElapsedTime eTime = new ElapsedTime();

    @Override
    public void runOpMode()
    {
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        servo = hardwareMap.get(Servo.class, "servo");

        flywheel_reader = new EncoderReader(flywheel, 28, 0.1);

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
    public void controls() {
        // EncoderRead();
        double rpm = 0; // flywheel_reader.readCycle();
        telemetry.addData("Shooter RPM", rpm);

        launchEm(rpm);
        telemetry.update();
    }
    public void launchEm(double rpm) {
        flywheel.setPower(gamepad1.right_trigger);

        if (rpm > 5500 || gamepad1.a) {
            triggered = true;
        }
        if (triggered = true) {
            servo.setPosition(0.44);
            if (eTime.time() > 1) {
                servo.setPosition(0);
                eTime.reset();
                triggered = false;
            }
        }
    }
}