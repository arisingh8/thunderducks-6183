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
import org.firstinspires.ftc.robotcore.internal.usb.exception.RobotUsbWriteLockException;
import org.firstinspires.ftc.teamcode.ChiefKeef.EncoderReader;

@Config
public class Drivetrain {
    private DcMotor flMotor, frMotor, rlMotor, rrMotor;
    private BNO055IMU imu;
    private Telemetry telemetry;

    private double g1lx = 0, g1ly = 0, g1rx = 0;
    private boolean g1x, g1lbumper;

    public static double P = 0.01;
    public static double I = 0.01;
    public static double D = 0;

    private double integral, previous_error = 0;

    private double FL_power_raw, FR_power_raw, RL_power_raw, RR_power_raw;
    private double FL_power, FR_power, RL_power, RR_power;

    private double newForward, newStrafe;

    private EncoderReader FL_reader;

    private Orientation angles;

    private double error;
    private double errorMin;
    private double desiredAngle;

    private ElapsedTime eTime = new ElapsedTime();
    
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

        eTime.reset();
    }

    public void controls() {
        // EncoderRead();
        double rpms = FL_reader.readCycle();
        telemetry.addData("RPMs", rpms);

        holonomicFormula(false);
        setDriveChainPower();
    }

    public void drive(double g1lx, double g1ly, double g1rx, boolean g1lbumper, boolean g1x, boolean turn, Telemetry telemetry) {
        this.g1x = g1x;
        this.g1lx = g1lx;
        this.g1ly = g1ly;
        this.g1rx = g1rx;
        this.g1lbumper = g1lbumper;

        this.telemetry = telemetry;

        if (!turn) {
            controls();
        } else {
            turn();
        }
    }

    public void turn() {
        holonomicFormula(true);
        setDriveChainPower();
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

    public void holonomicFormula(boolean turn) {
        double gain = 4;
        double time = eTime.time();

        if (!turn) {
            getJoyValues();

            if (g1x) {
                desiredAngle = 0;
            }
            desiredAngle = desiredAngle + -(gain * g1rx);
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

            telemetry.addData("errorMin", errorMin);
            telemetry.addData("Turn Error", error);

            integral += (error*time); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
            eTime.reset();

            double derivative = (error - previous_error) / time;
            double rcw = P*error + I*integral + D*derivative;

            previous_error = error;

            telemetry.addData("Read angle", angles.firstAngle);
            telemetry.addData("Desired Angle", desiredAngle);

            FL_power_raw = -newForward + newStrafe - rcw;
            FR_power_raw = -newForward - newStrafe + rcw;
            RL_power_raw = -newForward - newStrafe - rcw;
            RR_power_raw = -newForward + newStrafe + rcw;

            FL_power = Range.clip(FL_power_raw, -1, 1);
            FR_power = Range.clip(FR_power_raw, -1, 1);
            RL_power = Range.clip(RL_power_raw,-1 ,1);
            RR_power = Range.clip(RR_power_raw, -1, 1);

            if (this.g1lbumper) {
                FL_power /= 4;
                FR_power /= 4;
                RL_power /= 4;
                RR_power /= 4;
            }
        } else {
            FL_power = 0.25;
            FR_power = -0.25;
            RL_power = 0.25;
            RR_power = -0.25;
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
