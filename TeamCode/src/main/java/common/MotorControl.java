package common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This class controls a motor on a separate thread to avoid blocking.
 */
public class MotorControl extends Thread {

    public enum MOTOR_STATE {IDLE, MOVING, MOVING_TO_POSITION }
    private MOTOR_STATE state = MOTOR_STATE.IDLE;
    
    private final DcMotor motor;
    LinearOpMode opMode;

    private int targetPosition = 0;
    private int minPosition = 0;
    private int maxPosition = 0;
    private int lowSpeedThreshold = 200;
    private double speed = 0.5;
    private double lowSpeed = 0.2;

    public MotorControl(LinearOpMode opMode, DcMotor motor) {

        this.opMode = opMode;
        this.motor = motor;
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Control the motor on a separate thread to avoid blocking
     */
    public void run() {

        Logger.message("motor control thread started for %s", this.getName());

        while (!opMode.isStarted()) Thread.yield();

        while (opMode.opModeIsActive()) {
            switch (state) {
                case IDLE:
                    Thread.yield();
                    continue;
                case MOVING:
                    limitRange();
                    continue;
                case MOVING_TO_POSITION:
                    runMotorToPosition();
                    state = MOTOR_STATE.IDLE;
            }
        }
        Logger.message("motor control thread stopped");
    }

    /**
     * Turn the motor on, if the motor is already on, change the speed of the motor
     *
     * @param speed  motor power (-1 to 1)
     */
    public void runMotor(double speed) {
        if (state != MOTOR_STATE.MOVING) {
            interruptAction();
        }
        if (inRange(speed)) {
            synchronized (motor) {
                motor.setPower(speed);
                state = MOTOR_STATE.MOVING;
            }
        }
    }

    /**
     * Stop the motor
     */
    public void stopMotor () {
        motor.setPower(0);
        state = MOTOR_STATE.IDLE;
    }

    /**
     * Run the motor to the specified encoder position.
     * @param position encoder position to run to
     * @param speed motor power (-1 to 1)
     * @param lowSpeed ramp down motor power
     */
    public void setPosition(int position, double speed, double lowSpeed) {

        interruptAction();
        synchronized (motor) {
            targetPosition = position;
            this.speed = speed;
            this.lowSpeed = lowSpeed;
            state = MOTOR_STATE.MOVING_TO_POSITION;
        }
    }

    /**
     * Set the maximum and minimum encoders positions of the motor
     *
     * @param minPosition maximum encoder tick
     * @param maxPosition minimum encoder tick
     */
    public void setRange (int minPosition, int maxPosition) {
        this.minPosition = minPosition;
        this.maxPosition = maxPosition;
    }

    public void setLowSpeedThreshold(int threshold) {
        lowSpeedThreshold = threshold;
    }

    /**
     * Check of the motor is moving.
     *
     * @return true if the motor is moving
     */
    public boolean motorIsBusy () {
        return state != MOTOR_STATE.IDLE;
    }

    private void interruptAction () {
        if (state != MOTOR_STATE.IDLE) {
            motor.setPower(0);
            state = MOTOR_STATE.IDLE;
        }
    }

    /**
     * Run the motor to the specified encoder position.
     */
    private void runMotorToPosition() {

        ElapsedTime elapsedTime = new ElapsedTime();

        int current = motor.getCurrentPosition();
        int last = current;
        int distance = Math.abs(current - targetPosition);
        Logger.message("run from %d to %d  distance %d at %4.2f", current, targetPosition, distance, speed);
        DcMotor.RunMode mode = motor.getMode();
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(speed);

        double lastMoveTime = 0;
        elapsedTime.reset();
        boolean fullPower = true;
        while (opMode.opModeIsActive() && state == MOTOR_STATE.MOVING_TO_POSITION) {

            current = motor.getCurrentPosition();
            int remaining = Math.abs(targetPosition - current);
            if (remaining < lowSpeedThreshold && speed != lowSpeed) {
                if (fullPower) {
                    motor.setPower(lowSpeed);
                    fullPower = false;
                    Logger.message("remaining %d set to lower speed", remaining);
                }
            }

            // if the motor has not moved for a while, kill the power
            if (current != last) {
                lastMoveTime = elapsedTime.milliseconds();
                last = current;
            } else if (elapsedTime.milliseconds() - lastMoveTime > 100) {
                Logger.message("motor not moving");
                break;
            }

            if (emergencyStop())
                break;
            if (!motor.isBusy()) {
                Logger.message("motor not busy");
                break;
            }

            //Logger.message("position %5d   remaining %5d  percent %5d  elapsed %6.2f ", current, remaining, (remaining*100/distance), elapsedTime.milliseconds());
        }
        Logger.message("current: %d target: %d", current, targetPosition);
        motor.setPower(0);
        motor.setMode(mode);
    }

    /**
     * Stop the motor is the encoder is at the boundary of the set range.
     */
    private void limitRange () {

        if (minPosition != maxPosition) {
            Logger.message("limiting motor range");
            double power = motor.getPower();
            while (opMode.opModeIsActive() && state == MOTOR_STATE.MOVING) {
                if (!inRange(power)) {
                    interruptAction();
                    break;
                }
            }
        }
    }

    /**
     * Check if the motor's encoder is within the set range.
     * @param power motor power (-1 to 1)
     * @return returns true if a range has been set and the encoder is
     *         within the range.
     */
    private boolean inRange(double power) {
        if (minPosition != maxPosition) {
            int position = motor.getCurrentPosition();
            if (power > 0 && position > maxPosition)
                return false;
            else if (power < 0 && position < minPosition)
                return false;
        }
        return true;
    }

    private boolean emergencyStop() {
        return opMode.gamepad1.back;
    }
}



