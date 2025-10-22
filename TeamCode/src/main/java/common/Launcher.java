package common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Arrays;
import java.util.List;

@com.acmerobotics.dashboard.config.Config

public class Launcher extends Thread {

    public static double pidP = 40.0;
    public static double pidI = 1.0;

    public static double TRIGGER_COCK   = 0.50;
    public static double TRIGGER_FIRE   = 0.25;

    public static double LOADER_HOLD    = 0.32;
    public static double LOADER_RELEASE = 0.50;
    public static long   LOADER_REACT_TIME = 100;               // time in millisecond for the loader to open/close

   private boolean loaderOpen = true;

    private enum LAUNCHER_STATE {IDLE, FIRE }
    private LAUNCHER_STATE state = LAUNCHER_STATE.IDLE;

    private final double MOTOR_RPM = 6000;                      // Gobilda Yellow Jacket Motor 5203-2402-0001
    private final double MOTOR_TICKS_PER_REV = 28;              // Gobilda Yellow Jacket Motor 5203-2402-0001
    private final double MAX_VELOCITY = MOTOR_TICKS_PER_REV * MOTOR_RPM / 60;

    // Linear Servo HLS12-5050-6V
    private final double LINEAR_SERVO_SPEED = 30.9;             // mm per second
    private final double LINEAR_SERVO_STROKE = 50;              // in mm
    private final double LINEAR_SERVO_RADIUS = 100;             // mm from center of launcher rotation to servo base

    private final Servo linearServo;
    private final Servo trigger;
    private final Servo loader;

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
    public List<DcMotorEx> motors;

    LinearOpMode opMode;
    private double angle;
    private double speed = 0.20;
    private double velocity;
    private double angleAdjustTime;
    private boolean running = false;

    private long velocityCheckTime;
    private long startTime;

    public Launcher(LinearOpMode opMode) {

        this.opMode = opMode;

        leftMotor = opMode.hardwareMap.get(DcMotorEx.class, Config.LAUNCHER_LEFT);
        rightMotor = opMode.hardwareMap.get(DcMotorEx.class, Config.LAUNCHER_RIGHT);
        motors = Arrays.asList(leftMotor, rightMotor);

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // adjust the PID coefficients of the launcher motors so the the motors spin at the desired speed faster
        // and more accurately.
        PIDFCoefficients coefficients =  leftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        Logger.message("coefficients: %s", coefficients.toString());
        coefficients.p = pidP;
        coefficients.i = pidI;

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
            Logger.message("coefficients: %s", coefficients.toString());
        }

        linearServo = opMode.hardwareMap.get(Servo.class, Config.LINEAR_SERVO);

        trigger = opMode.hardwareMap.get(Servo.class, Config.TRIGGER);
        loader = opMode.hardwareMap.get(Servo.class, Config.LOADER);

        this.setName("launcher");
    }

    /**
     * Control the launcher on a separate thread to avoid blocking
     */
    public void run() {

        Logger.message("launcher thread started for %s", this.getName());

        while (!opMode.isStarted()) Thread.yield();

        while (opMode.opModeIsActive()) {
            switch (state) {
                case IDLE:
                    checkVelocity();
                    Thread.yield();
                    continue;
                case FIRE:
                    fire();
                    state = LAUNCHER_STATE.IDLE;
            }
        }
        Logger.message("launcher control thread stopped");
    }

    /**
     * Turn the launcher on, if the launcher is already on change the speed of the launcher motors
     */
    public void runLauncher() {
        Logger.message("launcher run");
        running = true;
        setSpeed(speed);
        startTime = System.currentTimeMillis();
    }

    /**
     * Stop the launcher motors
     */
    public void stopLauncher () {
        Logger.message("launcher stop");
        if (state != LAUNCHER_STATE.IDLE) {
            interruptAction();
        }
        running = false;
        setSpeed(0);
    }

    public void fireLauncher() {
        Logger.message("launcher fire");
        synchronized (this) {
            if (state == LAUNCHER_STATE.IDLE) {
                state = LAUNCHER_STATE.FIRE;
            }
        }
    }

    public void setSpeed(double speed) {
        synchronized (this) {
            this.speed = speed;
            velocity = MAX_VELOCITY * speed;
            if (running || speed == 0) {
                leftMotor.setVelocity(velocity);
                rightMotor.setVelocity(velocity);
            }
            Logger.message("launcher speed set to %f  velocity set to %f", speed, velocity);
        }
    }

    public void setAngle(double angle) {

        // Determine the distance to extend or retract the the linear servo.
        // Compare the length of the two chords formed by the  distance of the base of linear servo to the center
        // of rotation of the launcher and the current and new launcher angles.
        double extension = 2 * LINEAR_SERVO_RADIUS * Math.sin(Math.toRadians(this.angle / 2));
        double newExtension = 2 * LINEAR_SERVO_RADIUS * Math.sin(Math.toRadians(angle / 2));
        double delta = (newExtension - extension) / LINEAR_SERVO_STROKE;

        // get the current position of the linear servo
        double position = linearServo.getPosition();
        position += delta;

        // set the new position of the linear servo
        linearServo.setPosition(position);

        // calculate the time to move the linear servo
        long time = (long) ((Math.abs(delta) / LINEAR_SERVO_SPEED) * 1000);
        angleAdjustTime = System.currentTimeMillis() + time;
    }

    private void fire() {

        long timeout = 1000;
        long startTime = System.currentTimeMillis();

        // hold other artifacts
        loader.setPosition(LOADER_HOLD);

        // wait for the launcher to reach the desired launch angle.
        while (true) {

            long time = System.currentTimeMillis();
            if (time >= angleAdjustTime)
                break;

            if (time - startTime >= timeout)  {
                Logger.warning("launcher angle adjust timeout");
                break;
            }
        }

        // wait for the motors to spin up to the desired speed.
        while (true) {
            double leftVelocity = leftMotor.getVelocity();
            double rightVelocity = rightMotor.getVelocity();

            double threshold = MAX_VELOCITY * 0.01;
            if (Math.abs(velocity - Math.abs(leftVelocity)) < threshold ||
                    Math.abs(velocity - Math.abs(rightVelocity)) < threshold) {
                Logger.message("launcher spin up complete  left: %5.0f  right: %5.0f", leftVelocity, rightVelocity);
                break;
            }

            if (System.currentTimeMillis() - startTime >= timeout)  {
                Logger.warning("launcher spin up timed out after %d ms", timeout);
                break;
            }
        }

        // wait for the loader to close if it hasn't already.
        while (true) {
            long time = System.currentTimeMillis();
            if (time - startTime >= LOADER_REACT_TIME) {
                break;
            }
        }

        // pull the trigger
        trigger.setPosition(TRIGGER_FIRE);
        delay(500);
        trigger.setPosition(TRIGGER_COCK);
        delay(100);

        loader.setPosition(LOADER_RELEASE);
        delay(LOADER_REACT_TIME);
    }

    public void closeLoader() {
        loader.setPosition(LOADER_HOLD);
        loaderOpen = false;
        delay(LOADER_REACT_TIME);
    }

    public void openLoader() {
        loader.setPosition(LOADER_RELEASE);
        loaderOpen = true;
        delay(LOADER_REACT_TIME);
    }

    public boolean loaderIsOpen() {
        return loaderOpen;
    }

    public void pullTrigger() {
        // pull the trigger
        trigger.setPosition(TRIGGER_FIRE);
        delay(500);
        trigger.setPosition(TRIGGER_COCK);
        delay(100);
    }

    public boolean isBusy() {
        return state != LAUNCHER_STATE.IDLE;
    }

    public boolean isRunning() {
        return running;
    }

    private void interruptAction () {
        if (state != LAUNCHER_STATE.IDLE) {
            leftMotor.setVelocity(0);
            rightMotor.setVelocity(0);
            state = LAUNCHER_STATE.IDLE;
        }
    }

    private void checkVelocity () {
        if (running) {
            // check the velocity of the launcher motors every 0.5 seconds
            if (System.currentTimeMillis() - velocityCheckTime >= 500) {
                velocityCheckTime = System.currentTimeMillis();
                double leftVelocity = Math.abs(leftMotor.getVelocity());
                double rightVelocity = Math.abs(rightMotor.getVelocity());
                double leftCurrent = leftMotor.getCurrent(CurrentUnit.MILLIAMPS);
                double rightCurrent = rightMotor.getCurrent(CurrentUnit.MILLIAMPS);
                Logger.message("velocity  left: %5.0f  right: %5.0f     current  left: %5.0f  right: %5.0f   time: %d",
                        leftVelocity, rightVelocity, leftCurrent, rightCurrent,
                        System.currentTimeMillis() - startTime);
            }
        }
    }

    private void delay (long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    private boolean emergencyStop() {
        return opMode.gamepad1.back;
    }
}



