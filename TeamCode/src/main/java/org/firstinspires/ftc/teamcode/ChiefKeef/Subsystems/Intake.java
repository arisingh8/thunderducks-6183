package org.firstinspires.ftc.teamcode.ChiefKeef.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ChiefKeef.EncoderReader;

public class Intake {
    private DcMotor intakeMotor1;
    private DcMotor intakeMotor2;
    private Telemetry telemetry;

    private boolean g1b;
    private boolean oldg1b;
    private boolean run;

    private EncoderReader intake1_reader;
    private EncoderReader intake2_reader;

    public void init(HardwareMap hardwareMap) {
        intakeMotor1 = hardwareMap.get(DcMotor.class, "intake1");
        intakeMotor2 = hardwareMap.get(DcMotor.class, "intake2");


        intake1_reader = new EncoderReader(intakeMotor1, 537.6, 0.1);
        intake2_reader = new EncoderReader(intakeMotor2, 537.6, 0.1);
    }

    public void controls() {
        double rpm1 = intake1_reader.readCycle();
        telemetry.addData("Intake 1 RPM", rpm1);

        double rpm2 = intake2_reader.readCycle();
        telemetry.addData("Intake 2 RPM", rpm2);

        if (g1b && !oldg1b) {
            run = !run;
        }
        oldg1b = g1b;
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
            intakeMotor2.setPower(-1);
        } else {
            intakeMotor1.setPower(0);
            intakeMotor2.setPower(0);
        }
    }
}
