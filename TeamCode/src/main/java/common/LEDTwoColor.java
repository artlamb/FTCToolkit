package common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LED;

public class LEDTwoColor {

    private final LED redLED;
    private final LED greenLED;
    private LEDColor color;
    private boolean on;
    private boolean blink;
    private long lastUpdate;

    public enum LEDColor { GREEN, RED, YELLOW, NONE }

    public LEDTwoColor(LinearOpMode opMode, String redLEDName, String greenLEDName) {

        greenLED = opMode.hardwareMap.get(LED.class, greenLEDName);
        redLED = opMode.hardwareMap.get(LED.class, redLEDName);
    }

    private void setLED (LEDColor color) {
        if (color == LEDColor.GREEN || color == LEDColor.NONE ) {
            redLED.off();
        }
        if (color == LEDColor.RED  || color == LEDColor.NONE) {
            greenLED.off();
        }
        if (color == LEDColor.GREEN || color == LEDColor.YELLOW ) {
            greenLED.on();
        }
        if (color == LEDColor.RED || color == LEDColor.YELLOW) {
            redLED.on();
        }
        on = color != LEDColor.NONE;
    }

    public void on(boolean on) {
        if (on) {
            setLED(color);
        } else {
            setLED(LEDColor.NONE);
        }
    }

    public void setColor(LEDColor color) {
        this.color = color;
        setLED(color);
    }

    public void blink(boolean blink) {
        this.blink = blink;
        on(!blink);
        lastUpdate = System.currentTimeMillis();;
    }

    public void update() {
        if (! blink) return;

        long time = System.currentTimeMillis();
        if (time - lastUpdate < 500) {
            return;
        }
        lastUpdate = time;
        on(! on);
    }
}

