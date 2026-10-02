package common;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

import java.util.Locale;

public class Floodgate {

    private static final double MAX_CURRENT = 80;
    private static final double MAX_VOLTAGE = 3.3;
    private static final long UPDATE_FREQUENCY = 25;

    private final AnalogInput currentSensor;
    private long updateTime;
    private double current;
    private double maxCurrent;

    public Floodgate(LinearOpMode opMode) {
        currentSensor = opMode.hardwareMap.get(AnalogInput.class, "floodgate");
    }

    /**
     * Get the current that the robot is consuming.
     */
    public double getCurrent() {
        double voltage = currentSensor.getVoltage();
        return (voltage / MAX_VOLTAGE) * MAX_CURRENT;
    }

    /**
     * Periodically update the max current that the robot is consuming.
     */
    public void update() {

        if (System.currentTimeMillis() - updateTime > UPDATE_FREQUENCY) {
            updateTime = System.currentTimeMillis();
            current = getCurrent();
            if (current > maxCurrent) {
                maxCurrent = current;
                Logger.verbose("Max current %f", maxCurrent);
            }
        }
    }

    /**
     * Return the current that the robot is consuming and the maximum current that it has ever consumed.
     */
    @NonNull
    public String toString() {
        return String.format(Locale.US,"current: %5.3f amps  max current: %5.3f amps", current, maxCurrent);
    }
}