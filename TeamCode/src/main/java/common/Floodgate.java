package common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

public class Floodgate {

    private final AnalogInput currentSensor;
    private long updateTime;
    private long displayTime;
    private double current;
    private double maxCurrent;


    public Floodgate(LinearOpMode opMode) {
        currentSensor = opMode.hardwareMap.get(AnalogInput.class, "currentSensor");

    }

    /**
     * Get the current that the robot is consuming.
     */
    public double getCurrent() {
        double maxVoltage = currentSensor.getMaxVoltage();
        double voltage = currentSensor.getVoltage();
        double current = 80 * (voltage / maxVoltage);
        //Logger.message("sensor value: %f  max: %f,  current: %f", voltage, maxVoltage, current );
        return 80 * (voltage / maxVoltage);  // 80 amps is the max measured current
    }

    /**
     * Periodically update the max current that the robot is consuming.
     */
    public void update() {

        if (System.currentTimeMillis() - updateTime > 25) {
            updateTime = System.currentTimeMillis();
            current = getCurrent();
            if (current > maxCurrent) {
                maxCurrent = current;
                Logger.verbose("current %f", current);
            }
        }
    }

    /**
     * Periodically display the current the robot is consuming.
     *
     * @param frequency the frequency in milliseconds to display the current
     */
    public void display(long frequency) {

        update();
        if (System.currentTimeMillis() - displayTime > frequency) {
            displayTime = System.currentTimeMillis();
            Logger.verbose("current %f  %f", current, maxCurrent);
        }
    }
}