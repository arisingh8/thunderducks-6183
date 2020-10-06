package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
//FTC Team 11848 | Spare Parts Robotics
public class MecanumWithGyroOpMode extends LinearOpMode
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

    private BNO055IMU imu;
    private DcMotor frTest;
    private DcMotor rrTest;
    private DcMotor flTest;
    private DcMotor rlTest;

    // State used for updating telemetry
    public Orientation lastAngles;
    public Orientation angles;
    public double globalAngle;
    
    public double[] globalCorrections;

    @Override
    public void runOpMode()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample op mode
        parameters.mode = BNO055IMU.SensorMode.IMU;
        // parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        frTest = hardwareMap.get(DcMotor.class, "frMotor");
        rrTest = hardwareMap.get(DcMotor.class, "rrMotor");
        flTest = hardwareMap.get(DcMotor.class, "flMotor");
        rlTest = hardwareMap.get(DcMotor.class, "rlMotor");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(null, null, 1000);

        Thread EncoderThread = new EncoderThread();

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        //Wait for the start button to be pressed.
        waitForStart();

        telemetry.addData("Mode", "running");
        EncoderThread.start();
        telemetry.update();
        sleep(1500);

        while (opModeIsActive())
        {
            controls();
        }
    }

    public void controls()
    {
        resetAngle();
        double angleCorrection = checkDirection();
        // double XaccelCorrection = checkXAcceleration();
        // double ZaccelCorrection = checkZAcceleration();

        holonomicFormula();
        setDriveChainPower(angleCorrection, globalCorrections);
        telemetry.update();
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
        telemetry.addData("Intended Z Angle", lastAngles.firstAngle);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Current Z Angle", angles.firstAngle);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        telemetry.addData("Deviation", globalAngle);

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    public double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .05;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /*
    public double checkXAcceleration()
    {
        new Velocity();
        Velocity acceleration = imu.getVelocity();
        double xVeloc = acceleration.xVeloc/1000;
        telemetry.addData("XVelocity", xVeloc);

        double XaccelerationCorrection = 0;
        double correctionFactor = 0.075;

        telemetry.addData("Leftstick Xpos", gamepad1.left_stick_x);
        if (gamepad1.left_stick_x == 0) {
            XaccelerationCorrection = correctionFactor * xVeloc;
        }

        return XaccelerationCorrection;
    }

    public double checkZAcceleration()
    {
        new Velocity();
        Velocity acceleration = imu.getVelocity();
        double zVeloc = acceleration.zVeloc/1000;
        telemetry.addData("ZVelocity", zVeloc);

        double ZaccelerationCorrection = 0;
        double correctionFactor = 0.075;

        telemetry.addData("Leftstick Ypos", gamepad1.left_stick_y);
        if (gamepad1.left_stick_y == 0) {
            ZaccelerationCorrection = correctionFactor * zVeloc;
        }

        return ZaccelerationCorrection;
    }
    */

    public class EncoderThread extends Thread
    {
        public void run()
        {
            telemetry.addData("EncoderThreadStatus", "running");
            
            correctionMethod();
        }
        public void correctionMethod() 
        {
            while (Thread.currentThread().isAlive()) {
                double[] motorDeltas = motorDeltas();
                getCorrections(motorDeltas);
            }
        }
        public double[] motorDeltas()
        {

            double flcurrent = flTest.getCurrentPosition();
            double frcurrent = frTest.getCurrentPosition();
            double rlcurrent = rlTest.getCurrentPosition();
            double rrcurrent = rrTest.getCurrentPosition();

            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            double flnew = flTest.getCurrentPosition();
            double frnew = frTest.getCurrentPosition();
            double rlnew = rlTest.getCurrentPosition();
            double rrnew = rrTest.getCurrentPosition();

            double fldelta = flnew - flcurrent;
            double frdelta = frnew - frcurrent;
            double rldelta = rlnew - rlcurrent;
            double rrdelta = rrnew - rrcurrent;

            double[] motorDeltas = {fldelta, frdelta, rldelta, rrdelta};
            telemetry.addData("MotorDeltas", fldelta);
            // telemetry.update();
            return motorDeltas;
        }
        public void getCorrections(double[] motorDeltas) {
            double flcorrection = -motorDeltas[0];
            double frcorrection = -motorDeltas[1];
            double rlcorrection = -motorDeltas[2];
            double rrcorrection = -motorDeltas[3];

            double gain = 0.0075;
            if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0) {
                double flpower = flcorrection * gain;
                double frpower = frcorrection * gain;
                double rlpower = rlcorrection * gain;
                double rrpower = rrcorrection * gain;

                double[] motorCorrections = {flpower, frpower, rlpower, rrpower};
                globalCorrections = motorCorrections;
            }
        }
    }

    public void getJoyValues()
    {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        leftStickY = gamepad1.left_stick_y;
        leftStickX = gamepad1.left_stick_x;
        rightStickX = gamepad1.right_stick_x;

        float pi = 3.1415926f;

        float gyro_degrees = angles.firstAngle;
        float gyro_radians = gyro_degrees * pi/180;
        newForward = leftStickY * Math.cos(gyro_radians) + leftStickX * Math.sin(gyro_radians);
        newStrafe = -leftStickY * Math.sin(gyro_radians) + leftStickX * Math.cos(gyro_radians);
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

    public void setDriveChainPower(double angleCorrection, double[] motorCorrections)
    {
        flTest.setPower(FL_power+angleCorrection+motorCorrections[0]);
        frTest.setPower(-FR_power-angleCorrection+motorCorrections[1]);
        rlTest.setPower(RL_power+angleCorrection+motorCorrections[2]);
        rrTest.setPower(-RR_power-angleCorrection+motorCorrections[3]);
    }
}