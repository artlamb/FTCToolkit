package common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

public class Floodgate {

    AnalogInput currentSensor;

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
        Logger.message("sensor value: %f  max: %f,  current: %f", voltage, maxVoltage, current );
        return 80 * (voltage / maxVoltage);  // 80 amps is the max measured current
    }
}