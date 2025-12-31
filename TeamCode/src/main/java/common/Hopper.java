package common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Hopper {

    private final Servo lever;
    private boolean open = false;
    protected final double leverUpPosition = 0.7;
    protected double levelDownPosition = 0.345;

    public Hopper(LinearOpMode opMode) {
        lever = opMode.hardwareMap.get(Servo.class, Config.LEVER);
        leverDown();
    }

    public void leverDown() {
        lever.setPosition(levelDownPosition);
        open = true;
    }

    public void leverUp() {
        lever.setPosition(leverUpPosition);
        open = false;
    }

    /**
     * Open the lever if it is closed, otherwise close it.
     */
    public void leverToggle() {
        if (open) {
            leverUp();
        } else {
            leverDown();
        }
    }
}