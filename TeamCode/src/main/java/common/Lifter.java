package common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.List;

public class Lifter extends Thread {

    private enum LIFTER_STATE {IDLE, MOVING, MOVING_TO_POSITION }
    private LIFTER_STATE state = LIFTER_STATE.IDLE;

    private final double MOTOR_RPM = 60;                        // Gobilda Yellow Jacket Motor 5203-2402-0014
    private final double MOTOR_TICKS_PER_REV = 2786.2;          // Gobilda Yellow Jacket Motor  5203-2402-0100
    private final double MAX_VELOCITY = MOTOR_TICKS_PER_REV * MOTOR_RPM / 60;

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
    public List<DcMotorEx> motors;

    LinearOpMode opMode;

    private int targetPosition = 0;
    private int minPosition = 0;
    private int maxPosition = 0;
    private int lowSpeedThreshold = 200;
    private double highSpeed;
    private double lowSpeed;
    private double speed;

    public Lifter(LinearOpMode opMode) {

        this.opMode = opMode;

        leftMotor = opMode.hardwareMap.get(DcMotorEx.class, Config.LIFTER_LEFT);
        rightMotor = opMode.hardwareMap.get(DcMotorEx.class, Config.LIFTER_RIGHT);
        motors = Arrays.asList(leftMotor, rightMotor);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        this.setName("lifter");
    }

    public void resetEncoders () {
        for (DcMotor motor : motors) {
            DcMotor.RunMode mode = motor.getMode();
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(mode);
        }
    }

    /**
     * Control the lifter on a separate thread to avoid blocking
     */
    public void run() {

        Logger.message("lifter thread started for %s", this.getName());

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
                    runLifterToPosition();
                    state = LIFTER_STATE.IDLE;
            }
        }
        Logger.message("lifter control thread stopped");
    }

    /**
     * Turn the lifter on, if the lifter is already on change the speed of the lifter motors
     *
     * @param speed  motor power (-1 to 1)
     */
    public void runLifter(double speed) {
        if (state != LIFTER_STATE.MOVING) {
            interruptAction();
        }
        this.speed = speed;
        if (inRange()) {
            synchronized (this) {
                double velocity = MAX_VELOCITY * speed;
                leftMotor.setVelocity(velocity);
                rightMotor.setVelocity(velocity);
                state = LIFTER_STATE.MOVING;
            }
        }
    }

    /**
     * Stop the leftMotor
     */
    public void stopLifter () {
        leftMotor.setVelocity(0);
        rightMotor.setVelocity(0);
        state = LIFTER_STATE.IDLE;
    }

    public void setPosition(int position, double speed) {
        setPosition(position, speed, speed);
    }

    /**
     * Run the lifter to the specified encoder position.
     *
     * @param position encoder position to run to
     * @param highSpeed leftMotor power (-1 to 1)
     * @param lowSpeed ramp down leftMotor power
     */
    public void setPosition(int position, double highSpeed, double lowSpeed) {

        interruptAction();
        synchronized (this) {
            targetPosition = position;
            this.highSpeed = highSpeed;
            this.lowSpeed = lowSpeed;
            state = LIFTER_STATE.MOVING_TO_POSITION;
        }
    }

    public void setRange (int minPosition, int maxPosition) {
        this.minPosition = minPosition;
        this.maxPosition = maxPosition;
    }

    public void setLowSpeedThreshold(int threshold) {
        lowSpeedThreshold = threshold;
    }

    public boolean lifterIsBusy () {
        return state != LIFTER_STATE.IDLE;
    }

    public void lifterResetPosition() {

    }

    private void interruptAction () {
        if (state != LIFTER_STATE.IDLE) {
            leftMotor.setVelocity(0);
            rightMotor.setVelocity(0);
            state = LIFTER_STATE.IDLE;
        }
    }

    private void runLifterToPosition() {

        ElapsedTime elapsedTime = new ElapsedTime();

        int currentLeft = leftMotor.getCurrentPosition();
        int currentRight = rightMotor.getCurrentPosition();

        int lastLeft = currentLeft;
        int lastRight = currentRight;

        Logger.message("run from %d %d to %d at %4.2f", currentLeft, currentRight, targetPosition, highSpeed);

        DcMotor.RunMode modeLeft = leftMotor.getMode();
        DcMotor.RunMode modeRight = rightMotor.getMode();

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setTargetPosition(targetPosition);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        double velocity = MAX_VELOCITY * highSpeed;
        leftMotor.setVelocity(velocity);
        rightMotor.setVelocity(velocity);

        boolean fullPower = true;
        double lastMoveLeft = 0;
        double lastMoveRight = 0;
        elapsedTime.reset();

        while (opMode.opModeIsActive() && state == LIFTER_STATE.MOVING_TO_POSITION) {

            currentLeft = leftMotor.getCurrentPosition();
            currentRight = rightMotor.getCurrentPosition();

            if (highSpeed != lowSpeed && fullPower) {
                int current = Math.min(Math.abs(currentLeft), Math.abs(currentRight));
                int remaining = Math.abs(targetPosition - current);
                if (remaining < lowSpeedThreshold) {
                    velocity = MAX_VELOCITY * lowSpeed;
                    leftMotor.setVelocity(velocity);
                    rightMotor.setVelocity(velocity);
                    fullPower = false;
                    Logger.message("remaining %d set to lower speed", remaining);
                }
            }

            if (! (leftMotor.isBusy() && rightMotor.isBusy())) {
                Logger.message("left motors is busy: %b   right motor is busy: %b", leftMotor.isBusy(), rightMotor.isBusy());
                break;
            }

            // if the either motor has not moved for a while, kill the power
            if (currentLeft != lastLeft) {
                lastMoveLeft = elapsedTime.milliseconds();
                lastLeft = currentLeft;
            } else if (elapsedTime.milliseconds() - lastMoveLeft > 100) {
                Logger.message("left motor not moving");
                break;
            }

            if (currentRight != lastRight) {
                lastMoveRight = elapsedTime.milliseconds();
                lastRight = currentRight;
            } else if (elapsedTime.milliseconds() - lastMoveRight > 100) {
                Logger.message("right motor not moving");
                break;
            }

            if (emergencyStop())
                break;

            /*
            Logger.verbose("%s position left: %5d   position right: %5d   remaining left: %5d  remaining right: %5d  delta: %5d  elapsed %6.2f ",
                    state,
                    currentLeft,
                    currentRight,
                    Math.abs(targetPosition - currentLeft),
                    Math.abs(targetPosition - currentRight),
                    Math.abs(Math.abs(targetPosition - currentLeft) - Math.abs(targetPosition - currentRight)),
                    elapsedTime.milliseconds()/1000);
             */
        }

        leftMotor.setVelocity(0);
        rightMotor.setVelocity(0);
        leftMotor.setMode(modeLeft);
        rightMotor.setMode(modeRight);

        Logger.message("current: %d %d  target: %d", currentLeft, currentRight, targetPosition);
    }

    private void limitRange () {

        if (minPosition != maxPosition) {
            Logger.message("limiting leftMotor range");
            while (opMode.opModeIsActive() && state == LIFTER_STATE.MOVING) {
                if (!inRange()) {
                    interruptAction();
                    break;
                }
            }
        }
    }

    private boolean inRange() {

        if (minPosition != maxPosition) {
            Logger.verbose("speed: %4.2f  left: % 6d  right: % 6d  min: %6d   max: %6d", speed, leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition(), minPosition, maxPosition);
            for (DcMotorEx motor : motors) {
                int position = motor.getCurrentPosition();
                if (speed > 0 && position >= maxPosition)
                    return false;
                else if (speed < 0 && position <= minPosition)
                    return false;
            }
        }
        return true;
    }

    private boolean emergencyStop() {
        return opMode.gamepad1.back;
    }
}



