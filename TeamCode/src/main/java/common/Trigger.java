package common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

public class Trigger extends Thread {

    private final LinearOpMode opMode;
    private final CRServo triggerServo;

    private enum TRIGGER_STATE { IDLE, FIRING }
    private volatile TRIGGER_STATE state = TRIGGER_STATE.IDLE;
    private long startTime = 0;

    public Trigger(LinearOpMode opMode) {
        this.opMode = opMode;
        this.triggerServo = opMode.hardwareMap.get(CRServo.class, Config.TRIGGER);
    }

    /**
     * Control the trigger servo on a separate thread to avoid blocking
     */
    @Override
    public void run() {
        Logger.message("trigger thread started for %s", getName());

        while (!opMode.isStarted()) Thread.yield();

        while (opMode.opModeIsActive()) {
            synchronized (this) {
                while (state == TRIGGER_STATE.IDLE && opMode.opModeIsActive()) {
                    try {
                        this.wait(100);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                        return;
                    }
                }

                if (state == TRIGGER_STATE.FIRING) {
                    long elapsed = System.currentTimeMillis() - startTime;
                    long remaining = 3000 - elapsed;
                    if (remaining <= 0) {
                        triggerServo.setPower(0.0);
                        state = TRIGGER_STATE.IDLE;
                    } else {
                        try {
                            this.wait(remaining);
                        } catch (InterruptedException e) {
                            Thread.currentThread().interrupt();
                            return;
                        }
                    }
                }
            }
        }
        Logger.message("trigger control thread stopped");
    }

    /**
     * Returns true if the trigger servo is active, false otherwise
     */
    public boolean isRunning() {
        return state != TRIGGER_STATE.IDLE;
    }

    /**
     * Toggles the trigger servo on if idle, or off if active
     */
    public void toggle() {
        if (isRunning()) {
            off();
        } else {
            on();
        }
    }

    /**
     * Turn the trigger servo off and reset to IDLE
     */
    public void off() {
        synchronized (this) {
            triggerServo.setPower(0.0);
            state = TRIGGER_STATE.IDLE;
            this.notifyAll();
        }
        Logger.debug("trigger off");
    }

    /**
     * Turn the trigger servo on to start a firing sequence
     */
    public void on() {
        synchronized (this) {
            triggerServo.setPower(1.0);
            startTime = System.currentTimeMillis();
            state = TRIGGER_STATE.FIRING;
            this.notifyAll();
        }
        Logger.debug("trigger on");
    }

    public void reverse() {
        synchronized (this) {
            triggerServo.setPower(-1.0);
            startTime = System.currentTimeMillis();
            state = TRIGGER_STATE.FIRING;
            this.notifyAll();
        }
        Logger.debug("trigger on in reverse");
    }

}