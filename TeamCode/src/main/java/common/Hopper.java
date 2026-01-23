package common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.acmerobotics.dashboard.config.Config           // allows public static to be changed in TFC Dashboard

public class Hopper {

    public static double leverUpPosition = 0.380;
    public static double levelDownPosition = 0.830;

    private final Servo lever;
    private boolean leverDown = false;

    public Hopper(LinearOpMode opMode) {
        lever = opMode.hardwareMap.get(Servo.class, Config.LEVER);
        leverDown();
    }

    public void leverDown() {
        Logger.debug("lever down");
        lever.setPosition(levelDownPosition);
        leverDown = true;
    }

    public void leverUp() {
        Logger.debug("lever up");
        lever.setPosition(leverUpPosition);
        leverDown = false;
    }

    /**
     * Open the lever if it is closed, otherwise close it.
     */
    public void leverToggle() {
        if (leverDown) {
            leverUp();
        } else {
            leverDown();
        }
    }

    /**
     * Returns true if the lever is in the up position
     *
     * @return true if the lever is in the up position, false otherwise
     */
    public  boolean isLeverDown() {
        return leverDown;
    }
}