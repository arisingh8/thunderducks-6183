package org.firstinspires.ftc.teamcode.ChiefKeef.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WobbleGoalArm {
    private DcMotor arm;
    private Servo claw;

    private Telemetry telemetry;

    private boolean oldg1a;
    private boolean run = true;
    private boolean autoDrop = false;

    private boolean  g1dl, g1dr, g1dd, g1du, g1a, g1rb;
    public void init(HardwareMap hardwareMap) {
        arm = hardwareMap.get(DcMotor.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setTargetPosition(0);
        arm.setPower(1);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        claw.setPosition(0);
    }

    public void controls() {
        if (g1rb) {
            if (g1du) {
                arm.setTargetPosition(1316);
                autoDrop = false;
            }
            if (g1dl) {
                arm.setTargetPosition(1974);
                autoDrop = false;
            }
            if (g1dd) {
                arm.setTargetPosition(2632);
                autoDrop = false;
            }
            if (g1dr) {
                arm.setTargetPosition(0);
                run = true;
                autoDrop = false;
            }
        }

        if (g1a && claw.getPosition() == 0 && arm.getCurrentPosition() < 2200) {
            arm.setTargetPosition(1974);
            autoDrop = true;
        }

        if (arm.getCurrentPosition() > 1600 && autoDrop) {
            run = false;
        }

        if (g1a && !oldg1a && !autoDrop) {
            run = !run;
        }
        oldg1a = g1a;

        if (run) {
            claw.setPosition(0);
        } else {
            claw.setPosition(0.25);
        }

        telemetry.addData("wobbleGoalArm position", arm.getCurrentPosition());
    }

    public void pickUp(boolean g1dl, boolean g1dr, boolean g1dd, boolean g1du, boolean g1a, boolean g1rb, Telemetry telemetry) {
        this.telemetry = telemetry;

        this.g1a = g1a;
        this.g1rb = g1rb;
        this.g1dl = g1dl;
        this.g1dr = g1dr;
        this.g1dd = g1dd;
        this.g1du = g1du;

        controls();
    }
}
