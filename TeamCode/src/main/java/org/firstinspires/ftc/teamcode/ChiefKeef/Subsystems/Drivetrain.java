package org.firstinspires.ftc.teamcode.ChiefKeef.Subsystems;

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
import org.firstinspires.ftc.teamcode.ChiefKeef.EncoderReader;

public class Drivetrain {
    private DcMotor flMotor, frMotor, rlMotor, rrMotor;
    private BNO055IMU imu;
    private Telemetry telemetry;

    private double g1lx = 0, g1ly = 0, g1rx = 0;
    private boolean g1lbutton;

    private Orientation angles;

    public double FL_power_raw, FR_power_raw, RL_power_raw, RR_power_raw;
    private double FL_power, FR_power, RL_power, RR_power;

    private double newForward, newStrafe;

    private EncoderReader FL_reader;

    ElapsedTime eTime = new ElapsedTime();
    
    public void init(HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample op mode
        parameters.mode = BNO055IMU.SensorMode.IMU;
        // parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        rrMotor = hardwareMap.get(DcMotor.class, "rrMotor");
        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        rlMotor = hardwareMap.get(DcMotor.class, "rlMotor");

        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rlMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL_reader = new EncoderReader(flMotor, 537.6, 0.1);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(null, null, 1000);
    }

    public void controls() {
        // EncoderRead();
        double rpms = FL_reader.readCycle();
        telemetry.addData("RPMs", rpms);

        holonomicFormula();
        setDriveChainPower();
    }

    public void drive(double g1lx, double g1ly, double g1rx, boolean g1lbutton, Telemetry telemetry) {
        this.g1lx = g1lx;
        this.g1ly = g1ly;
        this.g1rx = g1rx;
        this.g1lbutton = g1lbutton;

        this.telemetry = telemetry;

        controls();
    }

    public void getJoyValues()
    {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        float pi = 3.1415926f;

        float gyro_degrees = angles.firstAngle;
        float gyro_radians = gyro_degrees * pi/180;
        newForward = this.g1ly * Math.cos(gyro_radians) + this.g1lx * Math.sin(gyro_radians);
        newStrafe = -this.g1ly * Math.sin(gyro_radians) + this.g1lx * Math.cos(gyro_radians);
    }

    public void holonomicFormula()
    {
        getJoyValues();

        FL_power_raw = -newForward - newStrafe + this.g1rx;
        FR_power_raw = -newForward + newStrafe - this.g1rx;
        RL_power_raw = newForward + newStrafe + this.g1rx;
        RR_power_raw = newForward - newStrafe - this.g1rx;

        FL_power = Range.clip(FL_power_raw, -1, 1);
        FR_power = Range.clip(FR_power_raw, -1, 1);
        RL_power = Range.clip(RL_power_raw,-1 ,1);
        RR_power = Range.clip(RR_power_raw, -1, 1);

        if (this.g1lbutton) {
            FL_power /= 4;
            FR_power /= 4;
            RL_power /= 4;
            RR_power /= 4;
        }
    }

    public void setDriveChainPower()
    {
        flMotor.setPower(FL_power);
        frMotor.setPower(-FR_power);
        rlMotor.setPower(-RL_power);
        rrMotor.setPower(RR_power);
    }
}
