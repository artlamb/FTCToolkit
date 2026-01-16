package common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Intake {

    private final DcMotorEx intake;
    private double speed = 0.9;
    private boolean running = false;

    public Intake(LinearOpMode opMode) {
        intake = opMode.hardwareMap.get(DcMotorEx.class, Config.INTAKE);

    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    /**
     * Returns true if the intake motor is running, false otherwise
     *
     * @return true if the intake motor is running, false otherwise
     * @noinspection unused
     */
    public boolean isRunning() {
        return running;
    }

    /**
     * Turn the intake motor on set to the saved speed, if it off, otherwise turn it off.
     *
     * @noinspection unused
     */
    public void intakeToggle() {
        if (running) {
            off();
        } else {
            on();
        }
    }

    /**
     * Turn the intake motor off
     */
    public void off() {
        Logger.debug("intake off");
        intake.setPower(0);
        running = false;
    }

    /**
     * Turn the intake motor on
     */
    public void on() {
        Logger.debug("intake on");
        intake.setPower(speed);
        running = true;
    }
}