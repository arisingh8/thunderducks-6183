package org.firstinspires.ftc.teamcode.ChiefKeef;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class EncoderReader {
    ElapsedTime eTime = new ElapsedTime();
    DcMotor motor;
    double cachedDelta = 0;

    double TICKS_PER_ROTATION = 537.6, E_TIME_REFRESH = 0.1;

    public EncoderReader(DcMotor motor) {
        this.motor = motor;
    }

    public double read() {
        double currentRead = motor.getCurrentPosition();

        if (eTime.time() > E_TIME_REFRESH) {
            double newRead = motor.getCurrentPosition();

            double delta = newRead - currentRead;
            cachedDelta = delta;

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
