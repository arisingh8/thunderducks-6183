package org.firstinspires.ftc.teamcode.ChiefKeef.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config
public class Drivetrain {
    private DcMotor flMotor, frMotor, rlMotor, rrMotor;
    private BNO055IMU imu;
    private Telemetry telemetry;

    private double g1lx = 0, g1ly = 0, g1rx = 0;
    private boolean g1lb, g1rb, g1dl, g1dr, g1dd, g1du;
    private boolean oldg1x = false;

    public static double P = 0.04;
    public static double I = 0;
    public static double D = 0;
    // public static double rsGain = 3;

    private double integral, previous_error = 0;

    private double FL_power_raw, FR_power_raw, RL_power_raw, RR_power_raw;
    private double FL_power, FR_power, RL_power, RR_power;

    private double newForward, newStrafe;

    private Orientation angles;

    private double error;
    private double errorMin;
    private double desiredAngle = 0, cachedDesiredAngle = 0;

    private enum driveMode {
        DRIVER_CONTROLLED,
        AUTO_CONTROL
    }

    private driveMode driveState = driveMode.DRIVER_CONTROLLED;

    private final ElapsedTime eTime = new ElapsedTime();

    public void init(HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.mode = BNO055IMU.SensorMode.IMU;

        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        rrMotor = hardwareMap.get(DcMotor.class, "rrMotor");
        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        rlMotor = hardwareMap.get(DcMotor.class, "rlMotor");

        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rlMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(null, null, 1000);

        eTime.reset();
    }

    public void controls() {
        holonomicFormula();
        eTime.reset();
    }

    public void drive(double g1lx, double g1ly, double g1rx, boolean g1lb, boolean g1rb, boolean g1dl,
                      boolean g1dr, boolean g1dd, boolean g1du, Telemetry telemetry) {
        this.g1rb = g1rb;
        this.g1lx = g1lx;
        this.g1ly = g1ly;
        this.g1rx = g1rx;
        this.g1lb = g1lb;

        this.g1dl = g1dl;
        this.g1dr = g1dr;
        this.g1dd = g1dd;
        this.g1du = g1du;

        this.telemetry = telemetry;

        controls();
    }

    public void holonomicFormula() {
        double time = eTime.time();

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        float pi = 3.1415926f;

        float gyro_degrees = angles.firstAngle;
        float gyro_radians = gyro_degrees * pi / 180;
        newForward = this.g1ly * Math.cos(gyro_radians) + this.g1lx * Math.sin(gyro_radians);
        newStrafe = -this.g1ly * Math.sin(gyro_radians) + this.g1lx * Math.cos(gyro_radians);

        if (!g1rb) {
            if (g1du) {
                desiredAngle = 0;
            }
            if (g1dr) {
                desiredAngle = 270;
            }
            if (g1dd) {
                desiredAngle = 180;
            }
            if (g1dl) {
                desiredAngle = 90;
            }
        }
        // desiredAngle = desiredAngle + -(rsGain * g1rx);
        if (desiredAngle < -180) {
            desiredAngle += 360;
        } else if (desiredAngle > 180) {
            desiredAngle -= 360;
        }

        if (desiredAngle - angles.firstAngle < 0) {
            errorMin = Math.min(Math.abs(desiredAngle - angles.firstAngle), Math.abs(desiredAngle - angles.firstAngle + 360));
        } else {
            errorMin = Math.min(Math.abs(desiredAngle - angles.firstAngle), Math.abs(desiredAngle - angles.firstAngle - 360));
        }

        if (errorMin == Math.abs(desiredAngle - angles.firstAngle)) {
            error = desiredAngle - angles.firstAngle;
        } else if (errorMin == Math.abs(desiredAngle - angles.firstAngle - 360)) {
            error = desiredAngle - angles.firstAngle - 360;
        } else if (errorMin == Math.abs(desiredAngle - angles.firstAngle + 360)) {
            error = desiredAngle - angles.firstAngle + 360;
        }

        telemetry.addData("Turn Error", error);

        integral += (error * time);
        eTime.reset();

        double derivative = (error - previous_error) / time;
        double rcw = P * error + I * integral + D * derivative;

        previous_error = error;

        telemetry.addData("Read angle", angles.firstAngle);
        telemetry.addData("Desired Angle", desiredAngle);

        if (desiredAngle != cachedDesiredAngle) {
            driveState = driveMode.AUTO_CONTROL;
        } else {
            driveState = driveMode.DRIVER_CONTROLLED;
        }

        switch (driveState) {
            case AUTO_CONTROL:
                FL_power_raw = -newForward + newStrafe - rcw;
                FR_power_raw = -newForward - newStrafe + rcw;
                RL_power_raw = -newForward - newStrafe - rcw;
                RR_power_raw = -newForward + newStrafe + rcw;
            case DRIVER_CONTROLLED:
                FL_power_raw = -newForward + newStrafe + g1rx;
                FR_power_raw = -newForward - newStrafe - g1rx;
                RL_power_raw = -newForward - newStrafe + g1rx;
                RR_power_raw = -newForward + newStrafe - g1rx;
        }

        /*
        if (g1rx != 0) {
            FL_power_raw = -newForward + newStrafe + g1rx;
            FR_power_raw = -newForward - newStrafe - g1rx;
            RL_power_raw = -newForward - newStrafe + g1rx;
            RR_power_raw = -newForward + newStrafe - g1rx;
            desiredAngle = angles.firstAngle;
        } else {
            FL_power_raw = -newForward + newStrafe - rcw;
            FR_power_raw = -newForward - newStrafe + rcw;
            RL_power_raw = -newForward - newStrafe - rcw;
            RR_power_raw = -newForward + newStrafe + rcw;
        }
         */

        FL_power = Range.clip(FL_power_raw, -1, 1);
        FR_power = Range.clip(FR_power_raw, -1, 1);
        RL_power = Range.clip(RL_power_raw, -1, 1);
        RR_power = Range.clip(RR_power_raw, -1, 1);

        if (this.g1lb) {
            FL_power /= 4;
            FR_power /= 4;
            RL_power /= 4;
            RR_power /= 4;
        }

        cachedDesiredAngle = desiredAngle;

        flMotor.setPower(-FL_power);
        frMotor.setPower(FR_power);
        rlMotor.setPower(-RL_power);
        rrMotor.setPower(RR_power);
    }
}
