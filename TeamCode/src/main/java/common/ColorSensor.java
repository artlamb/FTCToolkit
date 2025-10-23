package common;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@Disabled
public class ColorSensor {

    static final float COLOR_SENSOR_GAIN = 2.2F;

    public enum COLOR {RED, BLUE, WHITE }

    private COLOR color = COLOR.WHITE;
    private LED redLED;
    private LED greenLED;

    private float lastHue = 0;
    private float lastSaturation = 0;

    private NormalizedColorSensor colorSensor;

    public ColorSensor(LinearOpMode opMode) {

        try {
            colorSensor = opMode.hardwareMap.get(NormalizedColorSensor.class, Config.COLOR_SENSOR);
            colorSensor.setGain(COLOR_SENSOR_GAIN);
            greenLED = opMode.hardwareMap.get(LED.class, Config.LED_GREEN);
            redLED = opMode.hardwareMap.get(LED.class, Config.LED_RED);

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
     * Update the LEDs based on the color sensor current reading
     */
    public void update() {

        if (colorSensor == null || redLED == null || greenLED == null) return;

        boolean found = false;
        float[] hsvValues = new float[3];

        // Get the normalized colors from the sensor
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        // Convert to HSV color space
        Color.colorToHSV(colors.toColor(), hsvValues);
        float hue = hsvValues[0];
        float saturation = hsvValues[1];

        if (hue != lastHue || saturation != lastSaturation) {
            Logger.message("hue %f saturation %f", hue, saturation);
            lastHue = hue;
            lastSaturation = saturation;
        }

        if (color == COLOR.BLUE) {
            if (hue >= 190 && hue <= 230 && saturation >= .5) {
                Logger.message("blue found");
                found = true;
            }
        } else if (color == COLOR.RED) {
            if ((hue >= 0 && hue <= 90) && saturation >= .5) {
                Logger.message("red found");
                found = true;
            }
        } else if (color == COLOR.WHITE) {
            if (hue <= 10 && saturation >= .1) {
                Logger.message("white found");
                found = true;
            }
        }
        redLED.enableLight(!found);
        greenLED.enableLight(found);
    }
}