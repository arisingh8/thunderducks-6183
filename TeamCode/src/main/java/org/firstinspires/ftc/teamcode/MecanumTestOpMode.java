package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
//FTC Team 11848 | Spare Parts Robotics
public class MecanumTestOpMode extends LinearOpMode
{
    //Variables
    public double leftStickY;
    public double leftStickX;
    public double rightStickX;
    public double FL_power_raw;
    public double FR_power_raw;
    public double RL_power_raw;
    public double RR_power_raw;
    public double FL_power;
    public double FR_power;
    public double RL_power;
    public double RR_power;

    public double newForward;
    public double newStrafe;

    public double correction;

    private BNO055IMU imu;
    private DcMotor frTest;
    private DcMotor rrTest;
    private DcMotor flTest;
    private DcMotor rlTest;

    // State used for updating telemetry
    public Orientation lastAngles;
    public Orientation angles;
    public double globalAngle;
    public Acceleration gravity;

    @Override
    public void runOpMode()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        frTest = hardwareMap.get(DcMotor.class, "frMotor");
        rrTest = hardwareMap.get(DcMotor.class, "rrMotor");
        flTest = hardwareMap.get(DcMotor.class, "flMotor");
        rlTest = hardwareMap.get(DcMotor.class, "rlMotor");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //Wait for the start button to be pressed.
        waitForStart();

        while (opModeIsActive())
        {
            controls();
        }
    }

    public void controls()
    {
        holonomicFormula();
        setDriveChainPower();
    }

    public void getJoyValues()
    {
        leftStickY = gamepad1.left_stick_y;
        leftStickX = gamepad1.left_stick_x;
        rightStickX = gamepad1.right_stick_x;

        float pi = 3.1415926f;

        newForward = leftStickY;
        newStrafe = leftStickX;
    }

    public void holonomicFormula()
    {
        getJoyValues();

        FL_power_raw = newForward - newStrafe - rightStickX;
        FR_power_raw = newForward + newStrafe + rightStickX;
        RL_power_raw = newForward + newStrafe - rightStickX;
        RR_power_raw = newForward - newStrafe + rightStickX;

        FL_power = Range.clip(FL_power_raw, -1, 1);
        FR_power = Range.clip(FR_power_raw, -1, 1);
        RL_power = Range.clip(RL_power_raw,-1 ,1);
        RR_power = Range.clip(RR_power_raw, -1, 1);
    }

    public void setDriveChainPower()
    {
        flTest.setPower(FL_power);
        frTest.setPower(-FR_power);
        rlTest.setPower(RL_power);
        rrTest.setPower(-RR_power);
    }
}