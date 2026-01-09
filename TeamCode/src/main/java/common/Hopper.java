package common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Hopper {

    private final Servo lever;
    private boolean leverOpen = false;
    protected double leverUpPosition = 0.430;
    protected final double levelDownPosition = 0.830;

    public Hopper(LinearOpMode opMode) {
        lever = opMode.hardwareMap.get(Servo.class, Config.LEVER);
        leverDown();
    }

    public void leverDown() {
        lever.setPosition(levelDownPosition);
        leverOpen = true;
    }

    public void leverUp() {
        lever.setPosition(leverUpPosition);
        leverOpen = false;
    }

    /**
     * Open the lever if it is closed, otherwise close it.
     */
    public void leverToggle() {
        if (leverOpen) {
            leverUp();
        } else {
            leverDown();
        }
    }
}