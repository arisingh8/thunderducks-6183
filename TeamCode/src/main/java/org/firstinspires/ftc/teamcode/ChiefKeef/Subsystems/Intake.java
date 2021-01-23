package org.firstinspires.ftc.teamcode.ChiefKeef.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ChiefKeef.EncoderReader;

public class Intake {
    private DcMotor intakeMotor1, intakeMotor2;
    private Telemetry telemetry;

    private boolean g1b;
    private boolean g1y;
    private boolean oldg1b;
    private boolean run;

    private EncoderReader intake1_reader, intake2_reader;

    public void init(HardwareMap hardwareMap) {
        intakeMotor1 = hardwareMap.get(DcMotor.class, "intake1");
        intakeMotor2 = hardwareMap.get(DcMotor.class, "intake2");
    }

    public void controls() {
        if (g1b && !oldg1b) {
            run = !run;
        }
        oldg1b = g1b;

        runIntake();
    }

    public void intake(boolean g1b, boolean g1y, Telemetry telemetry) {
        this.g1b = g1b;
        this.g1y = g1y;
        this.telemetry = telemetry;

        controls();
    }

    public void runIntake() {
        if (g1y) {
            intakeMotor1.setPower(-1);
            intakeMotor2.setPower(-1);
        } else if (run) {
            intakeMotor1.setPower(1);
            intakeMotor2.setPower(1);
        } else {
            intakeMotor1.setPower(0);
            intakeMotor2.setPower(0);
        }
    }
}
