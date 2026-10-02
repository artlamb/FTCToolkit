package common;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

//@com.acmerobotics.dashboard.config.Config
@SuppressWarnings("unused")

public class ColorSensor {

    public static float COLOR_SENSOR_GAIN = 2.2F;

    public enum COLOR { RED, BLUE, WHITE, UNKNOWN }

    private COLOR color = COLOR.UNKNOWN;
    private float lastHue = 0;
    private float lastSaturation = 0;

    private NormalizedColorSensor colorSensor;

    public ColorSensor(LinearOpMode opMode) {

        try {
            colorSensor = opMode.hardwareMap.get(NormalizedColorSensor.class, Config.COLOR_SENSOR);
            colorSensor.setGain(COLOR_SENSOR_GAIN);
            Logger.message("color sensor installed");

        } catch (Exception e) {
            Logger.error(e, "Color sensor hardware not found");
        }
    }

    /**
     * Set the color to look for.
     *
     * @param color the color to look for
     */
    public void SetColor (COLOR color)  {
        this.color = color;
    }

    /**
     * Check if the sensor detects the specified color
     */
    public boolean findColor() {
        return findColor(color);
    }

    /**
     * Check if the sensor detects the specified color
     * @param color the color to look for
     */
    public boolean findColor(COLOR color) {

        if (colorSensor == null) {
            Logger.warning("color sensor not found");
            return false;
        }

        boolean found = false;
        float[] hsvValues = new float[3];

        // Get the normalized colors from the sensor
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        // Convert to HSV color space
        Color.colorToHSV(colors.toColor(), hsvValues);
        float hue = hsvValues[0];
        float saturation = hsvValues[1];

        if (hue != lastHue || saturation != lastSaturation) {
            Logger.verbose("hue %f saturation %f", hue, saturation);
            lastHue = hue;
            lastSaturation = saturation;
        }

        if (color == COLOR.BLUE) {
            if (hue >= 190 && hue <= 230 && saturation >= .5) {
                Logger.verbose("blue found");
                found = true;
            }
        } else if (color == COLOR.RED) {
            if ((hue >= 0 && hue <= 90) && saturation >= .5) {
                Logger.verbose("red found");
                found = true;
            }
        } else if (color == COLOR.WHITE) {
            if (hue <= 10 && saturation >= .1) {
                Logger.verbose("white found");
                found = true;
            }
        }
    return found;
    }
}