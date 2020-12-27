package org.firstinspires.ftc.teamcode.ChiefKeef.Subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ChiefKeef.EncoderReader;

public class Intake {
    private DcMotor intakeMotor1;
    private Telemetry telemetry;

    private boolean g1b;
    private boolean oldg1b;
    private boolean run;

    private EncoderReader intake1_reader;

    public void init(HardwareMap hardwareMap) {
        intakeMotor1 = hardwareMap.get(DcMotor.class, "intake1");

        intake1_reader = new EncoderReader(intakeMotor1, 537.6, 0.1);
        intakeMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void controls() {
        double rpm1 = intake1_reader.readCycle();
        telemetry.addData("Intake 1 RPM", rpm1);

        if (g1b && !oldg1b) {
            run = !run;
        }
        oldg1b = g1b;
/*
        if (colors.red > 0.008 && colors.red < 0.057 && colors.blue > 0.005 && colors.blue < 0.014 && colors.green > 0.011 && colors.green < 0.054) {
            colorSensorDetected = true;
        } else {
            run = false;
        }
 */
        runIntake();
    }

    public void intake(boolean g1b, Telemetry telemetry) {
        this.g1b = g1b;

        this.telemetry = telemetry;

        controls();
    }

    public void runIntake() {
        if (run) {
            intakeMotor1.setPower(1);
        } else {
            intakeMotor1.setPower(0);
        }
    }
}
