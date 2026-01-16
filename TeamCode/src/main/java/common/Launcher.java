package common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;
import java.util.List;

@com.acmerobotics.dashboard.config.Config

public class Launcher extends Thread {

    public static double pidP = 40.0;
    public static double pidI = 1.0;

    public static double TRIGGER_COCK   = 0.360;
    public static double TRIGGER_FIRE   = 0.670;

    public static double GATE_RIGHT_OPENED = 0.500;
    public static double GATE_RIGHT_CLOSED = 0.760;
    public static double GATE_LEFT_OPENED = 0.500;
    public static double GATE_LEFT_CLOSED = 0.240;

    public static long   GATE_REACT_TIME =    100;               // time in millisecond for the loader to open/close
    public static long   TRIGGER_FIRE_TIME =  300;               // time in millisecond to pull the trigger
    public static long   TRIGGER_COCK_TIME =  250;               // time in millisecond to cock the trigger
    public static long   ARTIFACT_LOAD_TIME = 500;

    public static double IDLE_SPEED = 20;

   private boolean gateOpen = true;

    private enum LAUNCHER_STATE {IDLE, FIRE, FIRE_ALL }
    private LAUNCHER_STATE state = LAUNCHER_STATE.IDLE;

    protected final double MOTOR_RPM = 6000;                      // Gobilda Yellow Jacket Motor 5203-2402-0001
    protected final double MOTOR_TICKS_PER_REV = 28;              // Gobilda Yellow Jacket Motor 5203-2402-0001
    protected final double MAX_VELOCITY = MOTOR_TICKS_PER_REV * MOTOR_RPM / 60;
    protected final double VELOCITY_MULTIPLIER = 20;

    // Linear Servo HLS12-5050-6V
    protected final double LINEAR_SERVO_SPEED = 30.9;             // mm per second
    protected final double LINEAR_SERVO_STROKE = 50;              // in mm
    protected final double LINEAR_SERVO_RADIUS = 100;             // mm from center of launcher rotation to servo base

    private final Servo linearServo;
    private final Servo trigger;
    private final Servo gateRight;
    private final Servo gateLeft;

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
    public List<DcMotorEx> motors;

    DistanceSensor artifactSensor;
    private long artifactCheckTime;

    LinearOpMode opMode;
    private double angle = 0;
    private double speed = 28;
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

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // adjust the PID coefficients of the launcher motors so the the motors spin at the desired speed faster
        // and more accurately.
        PIDFCoefficients coefficients =  leftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        Logger.message("coefficients: %s", coefficients.toString());
        coefficients.p = pidP;
        coefficients.i = pidI;

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
            Logger.message("coefficients: %s", coefficients.toString());
        }

        linearServo = opMode.hardwareMap.get(Servo.class, Config.LINEAR_SERVO);

        trigger = opMode.hardwareMap.get(Servo.class, Config.TRIGGER);
        trigger.setPosition(TRIGGER_COCK);

        gateRight = opMode.hardwareMap.get(Servo.class, Config.GATE_RIGHT);
        gateLeft = opMode.hardwareMap.get(Servo.class, Config.GATE_LEFT);
        gateOpen();

        artifactSensor = opMode.hardwareMap.get(DistanceSensor.class, Config.ARTIFACT_SENSOR);

        this.setName("launcher");
    }

    /**
     * Control the launcher on a separate thread to avoid blocking
     */
    public void run() {

        Logger.message("launcher thread started for %s", this.getName());

        while (!opMode.isStarted()) Thread.yield();

        while (opMode.opModeIsActive()) {
            synchronized (this) {
                switch (state) {
                    case IDLE:
                        //checkForArtifact();
                        Thread.yield();
                        break;
                    case FIRE:
                        fire();
                        state = LAUNCHER_STATE.IDLE;
                        break;
                    case FIRE_ALL:
                        fireAll();
                        state = LAUNCHER_STATE.IDLE;
                }
            }
        }
        Logger.message("launcher control thread stopped");
    }

    /**
     * Turn the launcher on, if the launcher is already on change the speed of the launcher motors
     */
    public void runLauncher() {
        Logger.message("launcher run");
            setVelocity(speed);
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
        setVelocity(0);
    }

    public void fireLauncher() {
        Logger.message("launcher fire");
        synchronized (this) {
            if (state == LAUNCHER_STATE.IDLE) {
                state = LAUNCHER_STATE.FIRE;
            }
        }
    }

    public void fireAllArtifacts() {
        Logger.message("launcher fire");
        synchronized (this) {
            if (state == LAUNCHER_STATE.IDLE) {
                state = LAUNCHER_STATE.FIRE_ALL;
            }
        }
    }

    public void setSpeed(double speed) {
        synchronized (this) {
            this.speed = speed;
            if (running) {
                setVelocity(speed);
            }
            Logger.message("launcher speed set to %f  velocity set to %f", speed, velocity);
        }
    }

    private void setVelocity(double speed) {
        //velocity = MAX_VELOCITY * speed;
        velocity = VELOCITY_MULTIPLIER * speed;
        leftMotor.setVelocity(velocity);
        rightMotor.setVelocity(velocity);
        running = (velocity != 0);
    }

    /**
     * Set the angle for the launcher, currently not used.
     *
     * @param angle angle in degrees
     * @noinspection unused
     */
    public void setAngle(double angle) {

        // Determine the distance to extend or retract the the linear servo.
        // Compare the length of the two chords formed by the  distance of the base of linear servo to the center
        // of rotation of the launcher and the current and new launcher angles.
        double extension = 2 * LINEAR_SERVO_RADIUS * Math.sin(Math.toRadians(this.angle / 2));
        double newExtension = 2 * LINEAR_SERVO_RADIUS * Math.sin(Math.toRadians(angle / 2));
        double delta = (newExtension - extension) / LINEAR_SERVO_STROKE;
        this.angle = angle;

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

        if (! Debug.launcher()) {
            Logger.message("launcher disabled");
            return;
        }

        long timeout = 3000;
        long startTime = System.currentTimeMillis();

        // wait for an artifact to load
        senseArtifact(ARTIFACT_LOAD_TIME);

        // hold other artifacts
        gateClose(0);

        // wait for the launcher to reach the desired launch angle.
        while (true) {

            long time = System.currentTimeMillis();
            if (time >= angleAdjustTime) {
                Logger.message("launcher angle adjust complete");
                break;
            }

            if (time - startTime >= timeout)  {
                Logger.warning("launcher angle adjust timeout");
                break;
            }
        }

        // wait for the motors to spin up to the desired speed.
        long spinUpStart = System.currentTimeMillis();
        while (true) {
            double leftVelocity = leftMotor.getVelocity();
            double rightVelocity = rightMotor.getVelocity();

            double threshold = MAX_VELOCITY * 0.001;
            if (Math.abs(velocity - Math.abs(leftVelocity)) <= threshold ||
                    Math.abs(velocity - Math.abs(rightVelocity)) <= threshold) {
                Logger.message("launcher spin up complete after %d ms  left: %5.0f  right: %5.0f ",
                        System.currentTimeMillis() - startTime, leftVelocity, rightVelocity);
                break;
            }

            if (System.currentTimeMillis() - startTime >= timeout)  {
                Logger.warning("launcher spin up timed out after %d ms", timeout);
                break;
            }

            Logger.verbose("time: %6d ms  delta velocity  left: %5.0f  right: %5.0f    current  left: %5.2f  right: %5.2f",
                    System.currentTimeMillis() - spinUpStart, velocity - leftVelocity, velocity - rightVelocity,
                    leftMotor.getCurrent(CurrentUnit.AMPS), rightMotor.getCurrent(CurrentUnit.AMPS));
            Thread.yield();
        }

        // wait for the loader to close if it hasn't already.
        while (true) {
            long time = System.currentTimeMillis();
            if (time - startTime >= GATE_REACT_TIME) {
                Logger.message("launcher loader close complete");
                break;
            }
        }

        // pull the trigger
        trigger.setPosition(TRIGGER_FIRE);
        delay(TRIGGER_FIRE_TIME);
        trigger.setPosition(TRIGGER_COCK);
        delay(TRIGGER_COCK_TIME);

        gateOpen(GATE_REACT_TIME);

        Logger.info("fire complete after %d ms", System.currentTimeMillis() - startTime);
    }

    private void fireAll() {
        long startTime = System.currentTimeMillis();
        setVelocity(speed);
        for (int i = 0; i < 3; i++) {
            fire();
            /*
            if (i < 2) {
                senseArtifact(ARTIFACT_LOAD_TIME);
                delay(ARTIFACT_LOAD_TIME);
            }
             */
        }
        setVelocity(IDLE_SPEED);   // todo determine current draw
        Logger.info("fire all complete after %d ms", System.currentTimeMillis() - startTime);
    }

    private void senseArtifact(long timeout) {
        long startTime = System.currentTimeMillis();
        while (true) {
            double distance = artifactSensor.getDistance(DistanceUnit.INCH);
            long time = System.currentTimeMillis();

            if (distance < 5) {
                Logger.debug("artifact detected after %d ms, distance: %5.1f", time - startTime, distance);
                break;
            }

            if (time - startTime >= timeout) {
                Logger.warning("artifact detection timed out after %d ms", timeout);
                break;
            }
            Thread.yield();
        }
    }

    public void gateClose() {
        gateClose(GATE_REACT_TIME);
    }

    public void gateOpen() {
        gateOpen(GATE_REACT_TIME);
    }

    public void gateClose(long delay) {
        Logger.debug("loader gate close");
        gateRight.setPosition(GATE_RIGHT_CLOSED);
        gateLeft.setPosition(GATE_LEFT_CLOSED);
        gateOpen = false;
        delay(delay);
    }

    public void gateOpen(long delay) {
        Logger.debug("loader gate open");
        gateRight.setPosition(GATE_RIGHT_OPENED);
        gateLeft.setPosition(GATE_LEFT_OPENED);
        gateOpen = true;
        delay(delay);
    }

    public boolean gateIsOpen() {
        return gateOpen;
    }

    public void gateToggle() {
        if (gateOpen) {
            gateClose();
        } else {
            gateOpen();
        }
    }

    public void pullTrigger() {
        // pull the trigger
        trigger.setPosition(TRIGGER_FIRE);
        delay(TRIGGER_FIRE_TIME);
        trigger.setPosition(TRIGGER_COCK);
        delay(TRIGGER_COCK_TIME);
    }

    public boolean isBusy() {
        return state != LAUNCHER_STATE.IDLE;
    }

    public boolean isRunning() {
        return running;
    }

    private void interruptAction () {
        if (state != LAUNCHER_STATE.IDLE) {
            setVelocity(0);
            state = LAUNCHER_STATE.IDLE;
        }
    }

    /**
     * Check the velocity of the launcher motors every 0.5 seconds
     *
     * @noinspection unused
     */
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

    /**
     * Check for the presence of artifacts every 0.5 seconds
     *
     * @noinspection unused
     */
    private void checkForArtifact () {
        // check for the presence of artifacts every 0.5 seconds
        long time = System.currentTimeMillis();
        if (time - artifactCheckTime >= 500) {
            artifactCheckTime = time;
            Logger.message("artifact distance: %5.0f", artifactSensor.getDistance(DistanceUnit.INCH));
        }
    }

    private void delay (long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}



