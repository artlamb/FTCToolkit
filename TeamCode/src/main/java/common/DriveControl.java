package common;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.Locale;

import utils.Dashboard;
import utils.MathUtil;
import utils.Pose;
import utils.PIDFCoefficients;
import utils.PIDFController;

/**
 * This class controls the movement of the robot. It handle moving
 * with the gamepad joysticks, following paths and moving with a distance
 * sensor or color sensor.

 */
@SuppressLint("DefaultLocale")
@com.acmerobotics.dashboard.config.Config

public class DriveControl extends Thread {

    public static double MAX_SPEED       = 0.60;
    public static double MAX_STICK_SPEED = 0.90;
    public static double MAX_TURN_SPEED  = 0.50;
    public static double MIN_SPEED       = 0.05;
    public static double MIN_TURN_SPEED  = 0.10;

    public static double TOLERANCE_DISTANCE_FAST = 10;              // in inches
    public static double TOLERANCE_DISTANCE_SLOW = 0.5;

    public static double TOLERANCE_HEADING_FAST = 20;               // in degrees
    public static double TOLERANCE_HEADING_SLOW = 0.5;

    public static double TOLERANCE_MAGNITUDE_FAST = 40;             // in inches per second
    public static double TOLERANCE_MAGNITUDE_SLOW = 5;

    public static double TOLERANCE_ROTATION_FAST = 200;              // in degrees per second
    public static double TOLERANCE_ROTATION_SLOW = 12;

    public static double DECELERATION_DRIVE = 0.15;                 // drive deceleration (distance to stop / magnitude)
    public static double DECELERATION_TURN = 0.09;                   // turn deceleration (rotation to stop / heading velocity)

    public static double SLOW_DRIVE = 10;

    public static double PID_P_DRIVE_FAST = 0.15;
    public static double PID_P_DRIVE_SLOW = 0.1;
    public static double PID_P_TURN_FAST = 2;
    public static double PID_P_TURN_SLOW = 1.2;

    public static double PID_DRIVE_P    = 0.07;
    public static double PID_DRIVE_P_EXPONENT = 0.9;

    public static double PID_TURN_S = 0.09;
    public static double PID_TURN_S_EXPONENT = 0.34;
    public static double PID_TURN_S_ERROR = 2;
    public static double PID_TURN_S_THRESHOLD = 3;

    private final PIDFCoefficients PIDFCoefficientsDriveFast = new PIDFCoefficients(
            PID_P_DRIVE_FAST, 0, 0, 0);

    private final PIDFCoefficients PIDFCoefficientsDriveSlow = new PIDFCoefficients(
            PID_P_DRIVE_SLOW, 1, 0, 0, 0, 0);

    private final PIDFCoefficients PIDFCoefficientsTurnFast = new PIDFCoefficients(
            PID_P_TURN_FAST, 0, 0, 0);

    private final PIDFCoefficients PIDFCoefficientsTurnSlow = new PIDFCoefficients(
            PID_P_TURN_SLOW, 1, PID_TURN_S, PID_TURN_S_EXPONENT, Math.toRadians(PID_TURN_S_ERROR), Math.toRadians(PID_TURN_S_THRESHOLD));

    private final PIDFCoefficients PIDFCoefficientsDrive = new PIDFCoefficients(
            PID_DRIVE_P, PID_DRIVE_P_EXPONENT,0, 0, 0, 0);

    PIDFController drivePID = new PIDFController(PIDFCoefficientsDriveFast);
    PIDFController turnPID = new PIDFController(PIDFCoefficientsTurnFast);
    PIDFController testPID = new PIDFController(PIDFCoefficientsDrive);

    private double distanceTolerance;
    private double headingTolerance;
    private double magnitudeTolerance;
    private double rotationTolerance;
    private double timeout;

    private final ElapsedTime timeoutTimer;
    private final ElapsedTime timer = new ElapsedTime();

    private double maxSpeed;
    private double minSpeed;

    private double stopDistance;
    private double targetDegrees;

    private double leftX;
    private double leftY;
    private double rightX;

    private  Pose target;

    private boolean interruptAction = false;

    private volatile boolean nearPose = false;

    private ArrayList<Pose> poses;

    private enum DRIVE_STATE { IDLE, MOVING, MOVING_TO_POSE, MOVING_TO_OBJECT, TURN_BY, FOLLOW_PATH, STOPPING }
    private volatile DRIVE_STATE driveState;

    private final Drive drive;
    Pinpoint localizer;
    LinearOpMode opMode;
    VoltageSensor voltageSensor;
    private DistanceSensor leftFrontSensor;
    private DistanceSensor rightFrontSensor;
    private final Dashboard dashboard;

    public DriveControl(LinearOpMode opMode, Drive drive) {

        this.setName("driveControl");
        this.opMode = opMode;
        this.drive = drive;
        driveState = DRIVE_STATE.IDLE;
        timeoutTimer = new ElapsedTime();
        dashboard = new Dashboard();

        voltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();
        initDistanceSensors();

        try {
            localizer = new Pinpoint(opMode);
        } catch (Exception e) {
            Logger.error(e, "Odometry Sensor not found");
        }
    }

    private void initDistanceSensors() {

        try {
            leftFrontSensor = opMode.hardwareMap.get(DistanceSensor.class, Config.LEFT_FRONT_SENSOR);
        } catch (Exception e) {
            Logger.error(e, "Left distance sensor not found", 2);
        }

        try {
            rightFrontSensor = opMode.hardwareMap.get(DistanceSensor.class, Config.RIGHT_FRONT_SENSOR);
        } catch (Exception e) {
            Logger.error(e, "Right distance sensor not found", 2);
        }
    }

    /**
     * Control the robot's movement on a separate thread to avoid blocking
     */
    public void run() {

        Logger.message("driveControl thread started for %s", this.getName());

        try {

            runDriveControl();

        }  catch (Exception e) {
            Logger.error(e, "Exception");
            throw e;
        }

        Logger.message("driveControl thread stopped");
    }

    /**
     * Drive control state machine
     */
    private void runDriveControl() {

        while (!opMode.isStarted()) Thread.yield();

        while (opMode.opModeIsActive()) {
            switch (driveState) {
                case IDLE:
                    if (isEmergencyStop()) drive.stopRobot();
                    Thread.yield();
                    continue;

                case MOVING:
                    moveWithJoystick();
                    continue;

                case MOVING_TO_POSE:
                    moveToPose();
                    driveState = DRIVE_STATE.IDLE;
                    continue;

                case MOVING_TO_OBJECT:
                    moveToObject();
                    driveState = DRIVE_STATE.IDLE;
                    continue;

                case TURN_BY:
                    turnBy();
                    driveState = DRIVE_STATE.IDLE;
                    break;

                case FOLLOW_PATH:
                    followPath();
                    driveState = DRIVE_STATE.IDLE;
                    break;

                case STOPPING:
                    drive.setVelocity(0);
                    driveState = DRIVE_STATE.IDLE;
            }
        }
    }

    private void moveInit (boolean highSpeed) {

        if (highSpeed) {
            turnPID.setCoefficients(PIDFCoefficientsTurnFast);
            drivePID.setCoefficients(PIDFCoefficientsDriveFast);

            distanceTolerance = TOLERANCE_DISTANCE_FAST;
            headingTolerance = TOLERANCE_HEADING_FAST;
            magnitudeTolerance = TOLERANCE_MAGNITUDE_FAST;
            rotationTolerance = TOLERANCE_ROTATION_FAST;

            testPID.reset();
            turnPID.reset();
            drivePID.reset();
        } else {
            turnPID.setCoefficients(PIDFCoefficientsTurnSlow);
            drivePID.setCoefficients(PIDFCoefficientsDriveSlow);

            distanceTolerance = TOLERANCE_DISTANCE_SLOW;
            headingTolerance = TOLERANCE_HEADING_SLOW;
            magnitudeTolerance = TOLERANCE_MAGNITUDE_SLOW;
            rotationTolerance = TOLERANCE_ROTATION_SLOW;
        }
    }

    /**
     * Move to the specified coordinate and heading
     */
    private void moveToPose() {

        timeoutTimer.reset();

        dashboard.drawField();
        dashboard.addWaypoint(target);

        moveInit(true);
        moveToPose(target);

        moveInit(false);
        moveToPose(target);

        drive.stopRobot();
        Pose pose = getPose();
        dashboard.setPose(pose);
        dashboard.drawField();

        double x = pose.getX();
        double y =  pose.getY();
        double heading = Math.toDegrees(pose.getHeading());

        double targetX = target.getX();
        double targetY = target.getY();
        double targetHeading = Math.toDegrees(target.getHeading());

        Logger.info("%s   %s   %s   %s",
                String.format("to: x %3.0f y %3.0f heading %4.0f", targetX, targetY, targetHeading),
                String.format("pose: x %5.1f  y %5.1f  heading %5.1f", x, y, heading),
                String.format("error: x %5.1f  y %5.1f  heading %5.1f", Math.abs(targetX-x), Math.abs(targetY-y), Math.abs(targetHeading-heading)),
                String.format("time: %4.2f", timeoutTimer.seconds()));

        dashboard.drawField();
    }

    /**
     * Move to the specified pose.
     *
     * @param target target pose
     */
    private void moveToPose(Pose target) {

        Logger.message("target  x: %3.0f, y: %3.0f, heading: %4.0f  p: %6.3f %6.3f",
                target.getX(), target.getY(), target.getHeading(AngleUnit.DEGREES), drivePID.getCoefficients().P, turnPID.getCoefficients().P);

        double maxVelocity = drive.getMaxVelocity();

        // Looping until we move the desired pose
        while (opMode.opModeIsActive() && !interruptAction) {

            Pose current = getPose();
            double a = target.getX() - current.getX();
            double b = target.getY() - current.getY();
            double distance = Math.hypot(a, b);
            double rotation = AngleUnit.normalizeRadians(current.getHeading() - target.getHeading());
            double angle = AngleUnit.normalizeRadians(Math.atan2(b, a) - current.getHeading());   // angle is robot relative
            double sin = Math.sin(angle + (Math.PI / 4));
            double cos = Math.cos(angle + (Math.PI / 4));
            double magnitude  = Math.hypot(localizer.getVelocityX(), localizer.getVelocityY());
            double velocityAngle = Math.atan2(localizer.getVelocityY(), localizer.getVelocityX());
            double deltaAngle = AngleUnit.normalizeRadians(angle - velocityAngle);
            double rotationVelocity = -localizer.getVelocityHeading();

            // adjust pid input errors to compensate for deceleration from current velocities
            double distanceError = distance;
            if ((magnitude > SLOW_DRIVE) && (Math.abs(deltaAngle) < Math.PI/6)) {
                distanceError = Math.max(0, distanceError - ((magnitude) * DECELERATION_DRIVE));
            }

            double driveDeceleration = magnitude * DECELERATION_DRIVE;
            if (MathUtil.getSign(angle) != MathUtil.getSign(velocityAngle))
                driveDeceleration = -driveDeceleration;

            double turnDeceleration = rotationVelocity * DECELERATION_TURN;
            if (MathUtil.getSign(rotation) != MathUtil.getSign(rotationVelocity))
                turnDeceleration = -turnDeceleration;

            drivePID.updateError(distanceError);
            double power = drivePID.runPIDF();

            turnPID.updateError(rotation, turnDeceleration);
            double turn = turnPID.runPIDF();

            testPID.updateError(distance, driveDeceleration);

            boolean inRange = Math.abs(a) < distanceTolerance && Math.abs(b) < distanceTolerance;
            boolean onBearing = Math.abs(rotation) <= Math.toRadians(headingTolerance);
            boolean stopped = Math.abs(magnitude) <= magnitudeTolerance;
            boolean rotated = Math.abs(rotationVelocity) <= Math.toRadians(rotationTolerance);
            nearPose = (distance < 5) && (rotation < Math.PI / 18) && (magnitude <= 10) && (Math.abs(rotationVelocity) <= Math.toRadians(10));

            // Scale the wheel velocity factors such that they don't total more the one.
            if (power + Math.abs(turn) > maxSpeed) {
                double scale = (power + Math.abs(turn)) / maxSpeed;
                power /= scale;
                turn /= scale;
            }

            double max = Math.max(Math.abs(sin), Math.abs(cos));
            double leftFrontPower  = (power * (cos / max) + turn);
            double rightFrontPower = (power * (sin / max) - turn);
            double leftRearPower   = (power * (sin / max) + turn);
            double rightRearPower  = (power * (cos / max) - turn);

            drive.setVelocity(
                    leftFrontPower * maxVelocity,
                    rightFrontPower * maxVelocity,
                    leftRearPower * maxVelocity,
                    rightRearPower * maxVelocity);

            Logger.debug("%s",
                    String.format("time: %4.0f   ", timeoutTimer.milliseconds()) +
                    String.format("x %5.1f  y %5.1f  h %5.1f  ", current.getX(), current.getY(), current.getHeading(AngleUnit.DEGREES)) +
                    String.format("distance: %5.2f %5.2f  ", distance, driveDeceleration) +
                    String.format("heading: %6.1f %6.1f  ", Math.toDegrees(rotation), Math.toDegrees(turnDeceleration)) +
                    String.format("a: %5.1f  b: %5.1f  angle: %4.0f  sin: %5.2f  cos: %5.2f  ", a, b, Math.toDegrees(angle), sin, cos) +
                    String.format("power: %4.2f  turn: %5.2f  ", power, turn) +
                    String.format("wheels: %5.2f %5.2f %5.2f %5.2f  ", leftFrontPower, rightFrontPower, leftRearPower, rightRearPower) +
                    String.format("vm: %3.0f  va: %4.0f  vh: %4.0f  ", magnitude, Math.toDegrees(deltaAngle), Math.toDegrees(rotationVelocity)) +
                    //String.format("volts: %4.1f  ", voltageSensor.getVoltage()) +
                    String.format("%s%s%s%s%s  ", (nearPose) ? "n":" ", (inRange) ? "i":" ", (onBearing) ? "o":" " , (stopped) ? "s":" ", (rotated) ? "r":" " )  +
                    //String.format("dr: %6.1f %6.1f  ", drivePID.errorDerivative, Math.toDegrees(turnPID.errorDerivative)) +
                    //String.format("ss: %6.2f  ", Math.toDegrees(turnPID.error - turnPID.previousError)) +
                    //String.format("%6.3f  ", Math.toDegrees(turnPID.errorSteadyState)) +
                    String.format("%32s", testPID.pidToString(false)) +
                    drivePID.pidToString(false)
            );

            if (inRange && onBearing && stopped && rotated) {
                break;
            }

            if (timeout > 0 && timeoutTimer.milliseconds() >= timeout) {
                Logger.warning("timed out");
                break;
            }
        }
    }

    private void followPath() {
        dashboard.drawField();
        moveInit(true);
        timeoutTimer.reset();

        for (Pose pose : poses) {
            dashboard.addWaypoint(pose);
            moveToPose(pose);
            dashboard.drawField();
        }

        int count = poses.size();
        Pose last = poses.get(count-1);
        moveInit(false);
        moveToPose(last);

        drive.stopRobot();
        Pose pose = getPose();
        dashboard.setPose(pose);
        dashboard.drawField();

        double x = pose.getX();
        double y =  pose.getY();
        double heading = Math.toDegrees(pose.getHeading());

        double targetX = last.getX();
        double targetY = last.getY();
        double targetHeading = Math.toDegrees(last.getHeading());

        Logger.info("%s   %s   %s   %s",
                String.format("to: x %3.0f y %3.0f heading %4.0f", targetX, targetY, targetHeading),
                String.format("pose: x %5.1f  y %5.1f  heading %5.1f", x, y, heading),
                String.format("error: x %5.1f  y %5.1f  heading %5.1f", Math.abs(targetX-x), Math.abs(targetY-y), Math.abs(targetHeading-heading)),
                String.format("time: %4.2f", timeoutTimer.seconds()));
    }

    /**
     * Move to the specified distance from an object detected a distance sensor
     */
    private void moveToObject() {

        timeoutTimer.reset();

        double distance = drive.distanceToObject() - stopDistance;
        Pose pose = getPose();
        Pose target = new Pose(pose.getX() + distance, pose.getY(), pose.getHeading());

        moveInit(true);
        moveToPose(target);

        moveInit(false);
        moveToPose(target);

        Logger.message("time: %4.2f", timeoutTimer.seconds());
        drive.stopRobot();
    }

    private void turnBy() {

        timeoutTimer.reset();

        Pose pose = getPose();
        double heading = Pose.normalizeAngle(pose.getHeading() + Math.toRadians(targetDegrees));
        heading = Math.toDegrees(Pose.normalizeAngle(heading));
        Pose target = new Pose(pose.getX(), pose.getY(), heading);

        moveInit(true);
        moveToPose(target);

        moveInit(false);
        moveToPose(target);

        Logger.message("time: %4.2f", timeoutTimer.seconds());
        drive.stopRobot();
    }

    private void moveWithJoystick () {

        double x = leftX;
        double y = leftY;
        double turn = rightX;

        double maxVelocity = drive.getMaxVelocity();
        double angle = Math.atan2(y, x);
        double sin = Math.sin(angle - (Math.PI / 4));
        double cos = Math.cos(angle - (Math.PI / 4));
        double max = Math.max(Math.abs(sin), Math.abs(cos));
        double power = Math.hypot(x, y);

        if (power != 0) {
            power = Math.pow(Math.abs(Math.min(power, 1)), 3);  // exponential power curve for better low speed control
            double minPower = drive.getMinPower();
            double maxPower = drive.getMaxPower();
            power = power * (maxPower - minPower) + minPower;
            power = Math.max(drive.accelerationLimit(power), minPower);
            turn /= 3;                              // limit turn speed when drive in any direction

        } else  if (turn != 0) {
            // if only turning scale joystick value for turning only.
            double sign = (turn < 0) ? -1 : 1;
            double minTurn = drive.getMinTurnPower();
            double maxTurn = drive.getMaxTurnPower();
            turn = Math.pow(Math.min(Math.abs(turn), 1), 3);  // exponential power curve for better low speed control

            //turn = Math.pow(Math.abs(Math.min(turn, 1)), 3);
            turn = (turn * (maxTurn - minTurn)) + minTurn;
            turn *= sign;
        }

        double scale = 1;
        if (power != 0 &&(power + Math.abs(turn) > MAX_STICK_SPEED))
            scale = (power + Math.abs(turn)) / MAX_STICK_SPEED;

        double leftFrontPower  = (power * (cos/max) + turn) / scale;
        double rightFrontPower = (power * (sin/max) - turn) / scale;
        double leftRearPower   = (power * (sin/max) + turn) / scale;
        double rightRearPower  = (power * (cos/max) - turn) / scale;

        drive.setVelocity(
                leftFrontPower * maxVelocity,
                rightFrontPower * maxVelocity,
                leftRearPower * maxVelocity,
                rightRearPower * maxVelocity);

        Logger.message("%s",
                String.format("x: %5.2f  y: %5.2f  x2: %5.2f  ", x, y, rightX) +
                        String.format("angle: %5.2f (rad)  %4.0f (deg)  ", angle, Math.toDegrees(angle)) +
                        String.format("power: %4.2f  sin: %5.2f  cos: %5.2f  ", power, sin, cos) +
                        String.format("turn: %5.2f  ", turn) +
                        String.format("power: %4.2f  %4.2f  %4.2f  %4.2f   ", leftFrontPower, rightFrontPower, leftRearPower, rightRearPower) +
                        String.format("pos: %6d %6d %6d %6d  ", drive.leftFrontDrive.getCurrentPosition(), drive.rightFrontDrive.getCurrentPosition(), drive.leftBackDrive.getCurrentPosition(), drive.rightBackDrive.getCurrentPosition()) +
                        String.format("velocity: %4.0f %4.0f %4.0f %4.0f", drive.leftFrontDrive.getVelocity(), drive.rightFrontDrive.getVelocity(), drive.leftBackDrive.getVelocity(), drive.rightBackDrive.getVelocity())
        );
    }

    private void interruptAction () {
        if (driveState != DRIVE_STATE.IDLE) {
            Logger.warning("interrupted");
            interruptAction = true;
            while (driveState != DRIVE_STATE.IDLE)  {
                Thread.yield();
            }
            interruptAction = false;
        }
    }

    public void alignInCorner() {

        double CORNER_DISTANCE_X = 4;
        double CORNER_DISTANCE_Y = 4;
        double distanceX = 0;
        double distanceY = 0;
        double samples = 5;
        for (int i = 0; i < samples; i++) {
            distanceX += leftFrontSensor.getDistance(DistanceUnit.INCH);
            distanceY += rightFrontSensor.getDistance(DistanceUnit.INCH);
        }
        distanceX /= samples;
        distanceY /= samples;

        Pose pose = getPose();
        double x = pose.getX() - (distanceX - CORNER_DISTANCE_X);
        double y = pose.getY() + (distanceY - CORNER_DISTANCE_Y);

        moveToPose(x, y, Math.toDegrees(pose.getHeading()), 2000);
        Logger.message("distance x: %6.1f  y: %6.1f   from x: %5.1f  y: %5.1f    to x: %5.1f  y: %5.1f ", distanceX, distanceY,pose.getX(), pose.getY(), x, y);
    }

    private boolean isEmergencyStop() {
        return opMode.gamepad1.back;        // todo remove
    }

    public void emergencyStop() {
        synchronized (this) {
            interruptAction();
            drive.stopRobot();
            driveState = DRIVE_STATE.IDLE;
        }
    }

    public void moveToPose(double targetX, double targetY, double targetHeading, double maxSpeed, double timeout) {
        moveToPose(new Pose(targetX, targetY, targetHeading), maxSpeed, timeout);
    }

    public void moveToPose(double targetX, double targetY, double targetHeading, double timeout) {
        moveToPose(new Pose(targetX, targetY, targetHeading), timeout);
    }

    public void moveToPose(Pose target, double timeout) {
        moveToPose(target, MAX_SPEED, timeout);
    }

    public void moveToPose(Pose target, double maxSpeed, double timeout) {
        synchronized (this) {
            interruptAction();

            this.target = target;
            this.maxSpeed = maxSpeed;
            this.minSpeed = MIN_SPEED;
            this.timeout = timeout;
            driveState = DRIVE_STATE.MOVING_TO_POSE;
        }
    }

    public void followPath (ArrayList<Pose> poses, double timeout) {
        synchronized (this) {
            interruptAction();
            this.poses = poses;
            this.maxSpeed = MAX_SPEED;
            this.minSpeed = MIN_SPEED;
            this.timeout = timeout;
            driveState = DRIVE_STATE.FOLLOW_PATH;
        }
    }

    public void turnBy(double degrees, double timeout) {

        synchronized (this) {
            interruptAction();

            this.targetDegrees = degrees;
            this.maxSpeed = MAX_TURN_SPEED;
            this.minSpeed = MIN_TURN_SPEED;
            this.timeout = timeout;
            driveState = DRIVE_STATE.TURN_BY;
        }
    }

    /**
     * Move forward until the distance sensor detects an object at the specified distance
     *
     * @param stopDistance  distance for the object to stop
     * @param timeout timeout in milliseconds
     */
    public void moveToObject (double stopDistance, double timeout) {

        synchronized (this) {
            interruptAction();

            this.stopDistance = stopDistance;
            this.timeout = timeout;
            driveState = DRIVE_STATE.MOVING_TO_OBJECT;
        }
    }

    public void moveWithJoystick (double leftX, double leftY, double rightX) {

        synchronized (this) {
            if (driveState != DRIVE_STATE.MOVING) {
                interruptAction();
            }
            this.leftX = leftX;
            this.leftY = leftY;
            this.rightX = rightX;
            driveState = DRIVE_STATE.MOVING;
        }
    }

    public void stopMoving() {

        synchronized (this) {
            if (driveState != DRIVE_STATE.IDLE && driveState != DRIVE_STATE.MOVING) {
                interruptAction();
            }
            driveState = DRIVE_STATE.STOPPING;
        }
    }

    public void startMoving() {
        drive.accelerationReset();
    }

    public boolean isBusy () {
        return driveState != DRIVE_STATE.IDLE;
    }

    public Pose getPose() {
        //Logger.message("%-20s %-24s", Thread.currentThread().getName(), Logger.getCaller());
        localizer.update();
        Pose pose = localizer.getPose();
        dashboard.setPose(pose);
        //Logger.message("x: %5.1f  y:%5.1f", pose.getX(), pose.getY());
        return pose;
    }
    
    public void setPose(Pose pose) {
        localizer.setPose(pose);
        dashboard.setPose(pose);

    }

    public boolean nearPose() {
        if (driveState == DRIVE_STATE.IDLE) {
            return true;
        }
        return nearPose;
    }

    public void resetIMU() {
        localizer.resetIMU();
    }

    /**
     * Wait until the robot stops moving
     */
    public void waitUntilNotMoving () {
        Logger.message("waiting, driveControl is %b", isBusy());
        if (!isBusy() )
            Logger.warning("driveControl is not busy");
        timer.reset();
        while (isBusy() &&  opMode.opModeIsActive()) {
            if (timer.milliseconds() > 3000) {
                Logger.warning("driveControl timed out");
                break;
            }
        }
        Logger.message("done waiting, time: %5.0f", timer.milliseconds());
    }

    public void poseTest(Pose target) {

        moveInit(false);

        Logger.message("target  x: %3.0f, y: %3.0f, heading: %4.0f", target.getX(), target.getY(), target.getHeading(AngleUnit.DEGREES));

        Pose current = getPose();
        double a = target.getX() - current.getX();
        double b = target.getY() - current.getY();
        double angle = Math.atan2(b, a);
        double distance = Math.hypot(a, b);
        double sin = Math.sin(angle + (Math.PI / 4));
        double cos = Math.cos(angle + (Math.PI / 4));
        double headingError = AngleUnit.normalizeRadians(current.getHeading() - target.getHeading());

        turnPID.updateError(headingError);
        double turn = turnPID.runPIDF();

        drivePID.updateError(distance);
        double power = drivePID.runPIDF();

        // If the heading error is greater than 45 degrees then give the heading error greater weight
        if (Math.abs(headingError) < Math.toRadians(45))  {
            turn *= 2;
        }

        double scale = 1;
        if (power + Math.abs(turn) > maxSpeed) {
            scale = (power + Math.abs(turn)) / maxSpeed;
        }  else if (power + Math.abs(turn) < minSpeed) {
            scale = (power + Math.abs(turn)) / minSpeed;
        }

        double max = Math.max(Math.abs(sin), Math.abs(cos));
        double leftFrontPower  = (power * (cos / max) + turn) / scale;
        double rightFrontPower = (power * (sin / max) - turn) / scale;
        double leftRearPower   = (power * (sin / max) + turn) / scale;
        double rightRearPower  = (power * (cos / max) - turn) / scale;

        Logger.verbose("%s",
                String.format(Locale.US,"x: %-5.1f  y: %-5.1f  h: %-5.1f  ", current.getX(), current.getY(), current.getHeading(AngleUnit.DEGREES)) +
                        String.format(Locale.US, "a: %5.1f  b: %5.1f  distance: %5.2f  ", a, b, distance) +
                        String.format(Locale.US, "heading error: %6.1f  ", Math.toDegrees(headingError)) +
                        String.format("angle: %4.0f  ", Math.toDegrees(angle)) +
                        String.format("turn: %5.2f  power: %4.2f  sin: %5.2f  cos: %5.2f  ", turn, power, sin, cos) +
                        String.format("wheels: %5.2f  %5.2f  %5.2f  %5.2f   ", leftFrontPower, rightFrontPower, leftRearPower, rightRearPower));
    }

    public void decelerationTest (double percent, double heading, boolean turn) {

        ElapsedTime timer = new ElapsedTime();
        double maxVelocity = drive.getMaxVelocity();
        double targetVelocity = maxVelocity * percent;
        Logger.message("percent %5.0f   velocity %5.2f", percent*100, targetVelocity);

        double currentVelocity;
        double accelerationTime = 0;
        double velocity =  Math.min(maxVelocity, targetVelocity);

        if (turn) {
            drive.setVelocity(-velocity, velocity, -velocity, velocity);
        } else {
            double sin = Math.sin(heading + (Math.PI / 4));
            double cos = Math.cos(heading + (Math.PI / 4));
            double max = Math.max(Math.abs(sin), Math.abs(cos));
            double leftFrontVelocity  = (velocity * (cos / max));
            double rightFrontVelocity = (velocity * (sin / max));
            double leftRearVelocity   = (velocity * (sin / max));
            double rightRearVelocity  = (velocity * (cos / max));
            drive.setVelocity(
                    leftFrontVelocity * maxVelocity,
                    rightFrontVelocity * maxVelocity,
                    leftRearVelocity * maxVelocity,
                    rightRearVelocity * maxVelocity);
        }


        timer.reset();

        Pose start = null;
        Pose stop = null;
        double magnitude = 0;
        double angle = 0;
        double rotationVelocity = 0;
        boolean accelerate = true;
        boolean done = false;
        boolean powerOff = false;

        do {
            currentVelocity = drive.getCurrentVelocity();
            Pose pose = getPose();
            if (accelerate && currentVelocity >= targetVelocity) {
                drive.setVelocity(0);
                start = pose;
                accelerationTime = timer.seconds();
                magnitude  = Math.hypot(localizer.getVelocityX(), localizer.getVelocityY());
                rotationVelocity = localizer.getVelocityHeading();
                angle = Math.toDegrees(Math.atan2(localizer.getVelocityY(), localizer.getVelocityX()));
                accelerate = false;
                powerOff = true;

            } else if ((! accelerate) && (Math.abs(currentVelocity) <= 10)) {
                stop = getPose();
                done = true;
            }
            Logger.verbose("%s",
                    String.format("x %5.1f  y %5.1f  h %6.1f  ", pose.getX(), pose.getY(), pose.getHeading(AngleUnit.DEGREES)) +
                    String.format("time %5.2f  ", timer.seconds()) +
                    String.format("velocity %5.0f  ", currentVelocity) +
                    String.format("percent  %3.0f  ", currentVelocity/targetVelocity*100) +
                    String.format("vx %6.2f  ", localizer.getVelocityX()) +
                    String.format("vy %6.2f  ", localizer.getVelocityY()) +
                    String.format("vh %6.2f  ", Math.toDegrees(localizer.getVelocityHeading())) +
                    String.format("%s  ", powerOff ? "power off":""));
            powerOff = false;

        } while (! done);

        double distance = Math.hypot(stop.getX() - start.getX(), stop.getY() - start.getY());
        double rotation = AngleUnit.normalizeRadians(stop.getHeading() - start.getHeading());
        double decelerationTime = timer.seconds() - accelerationTime;
        Logger.info("%s",
                String.format("percent %5.0f  ", percent*100) +
                String.format("time %5.2f  ", timer.seconds()) +
                String.format("acceleration %5.2f  ", accelerationTime) +
                String.format("deceleration %5.2f  ", decelerationTime) +
                String.format("velocity %5.0f  ", targetVelocity) +
                String.format("angle %6.1f  ", angle) +
                String.format("magnitude %6.1f  ", magnitude) +
                String.format("deceleration distance %5.2f  ", distance) +
                String.format("coefficient %5.2f  ", (distance / magnitude)) +
                String.format("rotation %5.2f  ", Math.toDegrees(rotation)) +
                String.format("coefficient %5.2f", (rotation / rotationVelocity)));
    }
}



