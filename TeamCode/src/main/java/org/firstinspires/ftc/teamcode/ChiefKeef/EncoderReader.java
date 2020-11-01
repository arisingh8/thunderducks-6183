package org.firstinspires.ftc.teamcode.ChiefKeef;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class EncoderReader {
    ElapsedTime eTime = new ElapsedTime();
    DcMotor motor;
    double currentRead;
    double cachedDelta = 0;

    double TICKS_PER_ROTATION, E_TIME_REFRESH;

    public EncoderReader(DcMotor motor, double ticks, double refresh) {
        this.motor = motor;
        this.TICKS_PER_ROTATION = ticks;
        this.E_TIME_REFRESH = refresh;

        currentRead = motor.getCurrentPosition();

        eTime.reset();
    }

    public double read() {
        if (eTime.time() > E_TIME_REFRESH) {
            double newRead = motor.getCurrentPosition();

            double delta = newRead - currentRead;
            cachedDelta = delta;
            currentRead = motor.getCurrentPosition();

            eTime.reset();
            return delta;
        } else {
            return cachedDelta;
        }
    }

    public double convertToRpm(double readValue) {
        readValue /= TICKS_PER_ROTATION;
        readValue *= 60/E_TIME_REFRESH;

        return readValue;
    }

    public double readCycle() {
        double read = read();

        return convertToRpm(read);
    }
}
