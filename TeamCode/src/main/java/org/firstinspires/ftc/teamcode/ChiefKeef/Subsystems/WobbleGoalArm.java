package org.firstinspires.ftc.teamcode.ChiefKeef.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WobbleGoalArm {
    private DcMotor arm;

    private DigitalChannel touchSensor;

    private Telemetry telemetry;

    private boolean  g1dl, g1dr, g1dd, g1du;
    public void init(HardwareMap hardwareMap) {
        arm = hardwareMap.get(DcMotor.class, "arm");
        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        touchSensor.setMode(DigitalChannel.Mode.INPUT);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setTargetPosition(0);
        arm.setPower(1);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void controls() {
        if (g1dl) { arm.setTargetPosition(1316);}
        if (g1dr) { arm.setTargetPosition(1974);}
        if (g1dd) { arm.setTargetPosition(2632);}
        if (g1du) { arm.setTargetPosition(0);}

        telemetry.addData("Touch Sensor state", touchSensor.getState());
        telemetry.addData("wobbleGoalArm position", arm.getCurrentPosition());
    }

    public void pickUp(boolean g1dl, boolean g1dr, boolean g1dd, boolean g1du, Telemetry telemetry) {
        this.telemetry = telemetry;

        this.g1dl = g1dl;
        this.g1dr = g1dr;
        this.g1dd = g1dd;
        this.g1du = g1du;

        controls();
    }
}
