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

    private double[] globalDeltas;
    private double[] globalCorrections = new double[4];

    enum EncoderReadStatus {
        current,
        calculate
    }

    private EncoderReader FL_reader;

    public EncoderReadStatus readStatus = EncoderReadStatus.current;
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

        FL_reader = new EncoderReader(flMotor);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(null, null, 1000);
    }

    public void controls() {
        // EncoderRead();
        double rpms = FL_reader.readCycle();
        telemetry.addData("RPMs", rpms);

        holonomicFormula();
        setDriveChainPower(globalCorrections);
    }

    public void drive(double g1lx, double g1ly, double g1rx, boolean g1lbutton, Telemetry telemetry) {
        this.g1lx = g1lx;
        this.g1ly = g1ly;
        this.g1rx = g1rx;
        this.g1lbutton = g1lbutton;

        this.telemetry = telemetry;

        controls();
    }

    /*
    public void EncoderRead() {
        switch (readStatus) {
            case current:
                double flcurrent = flMotor.getCurrentPosition();
                double frcurrent = frMotor.getCurrentPosition();
                double rlcurrent = rlMotor.getCurrentPosition();
                double rrcurrent = rrMotor.getCurrentPosition();

                if (eTime.time() > 0.1) {
                    double flnew = flMotor.getCurrentPosition();
                    double frnew = frMotor.getCurrentPosition();
                    double rlnew = rlMotor.getCurrentPosition();
                    double rrnew = rrMotor.getCurrentPosition();

                    double fldelta = flnew - flcurrent;
                    double frdelta = frnew - frcurrent;
                    double rldelta = rlnew - rlcurrent;
                    double rrdelta = rrnew - rrcurrent;

                    double[] motorDeltas = {fldelta, frdelta, rldelta, rrdelta};
                    telemetry.addData("MotorDeltas", motorDeltas.toString());
                    // telemetry.update();
                    globalDeltas = motorDeltas;
                    readStatus = EncoderReadStatus.calculate;

                    eTime.reset();
                }
                break;
            case calculate:
                double flcorrection = -globalDeltas[0];
                double frcorrection = -globalDeltas[1];
                double rlcorrection = -globalDeltas[2];
                double rrcorrection = -globalDeltas[3];

                double gain = 0.0075;
                if (this.g1lx == 0 && this.g1ly == 0) {
                    double flpower = flcorrection * gain;
                    double frpower = frcorrection * gain;
                    double rlpower = rlcorrection * gain;
                    double rrpower = rrcorrection * gain;

                    double[] motorCorrections = {flpower, frpower, rlpower, rrpower};
                    globalCorrections = motorCorrections;
                }
                readStatus = EncoderReadStatus.current;
                eTime.reset();
        }
    }
    */

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

    public void setDriveChainPower(double[] motorCorrections)
    {
        flMotor.setPower(-FL_power+motorCorrections[0]);
        frMotor.setPower(FR_power+motorCorrections[1]);
        rlMotor.setPower(RL_power+motorCorrections[2]);
        rrMotor.setPower(-RR_power+motorCorrections[3]);
    }
}
