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

    public static boolean verbose = false;

    public static double MAX_SPEED       = 0.6;
    public static double MAX_STICK_SPEED = 0.90;
    public static double MAX_TURN_SPEED  = 0.5;
    public static double MIN_SPEED       = 0.10;
    public static double MIN_TURN_SPEED  = 0.100;

    public static double DISTANCE_TOLERANCE_HIGH_SPEED = 10;
    public static double DISTANCE_TOLERANCE_LOW_SPEED = 0.5;

    public static double HEADING_TOLERANCE_HIGH_SPEED = 20;
    public static double HEADING_TOLERANCE_LOW_SPEED = 0.5;

    public static double VELOCITY_TOLERANCE_LOW_SPEED = 0.10;
    public static double VELOCITY_COEFFICIENTS = 0.15;

    public static PIDFCoefficients driveHighSpeedPIDFCoefficients = new PIDFCoefficients(
            0.15, 0, 0, 0);

    public static PIDFCoefficients driveLowSpeedPIDFCoefficients = new PIDFCoefficients(
            0.1, 0, 0, 0);

    public static PIDFCoefficients headingHighSpeedPIDFCoefficients = new PIDFCoefficients(
            2.5, 0, 0, 0);

    public static PIDFCoefficients headingLowSpeedPIDFCoefficients = new PIDFCoefficients(
            1, 0, 0, 0);

    PIDFController drivePID  = new PIDFController(driveHighSpeedPIDFCoefficients);
    PIDFController headingPID = new PIDFController(headingHighSpeedPIDFCoefficients);

    private double distanceTolerance;
    private double headingTolerance;
    private double velocityTolerance;
    private double timeout;
    private final ElapsedTime timeoutTimer;

    private double targetX;
    private double targetY;
    private double targetHeading;

    private double maxSpeed;
    private double minSpeed;

    private double stopDistance;

    private double leftX;
    private double leftY;
    private double rightX;

    private boolean interruptAction = false;

    private boolean highSpeed;
    private volatile boolean nearPose = false;

    private ArrayList<Pose> poses;

    private enum DRIVE_STATE { IDLE, MOVING, MOVING_TO_COORDINATE, MOVING_TO_OBJECT, TURN_TO, FOLLOW_PATH, STOPPING }
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

                case MOVING_TO_COORDINATE:
                    moveToCoordinate(true);
                    driveState = DRIVE_STATE.IDLE;
                    continue;

                case MOVING_TO_OBJECT:
                    moveToObject();
                    driveState = DRIVE_STATE.IDLE;
                    continue;

                case TURN_TO:
                    moveToCoordinate(false);
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

        this.highSpeed = highSpeed;

        if (highSpeed) {
            headingPID.setCoefficients(headingHighSpeedPIDFCoefficients);
            drivePID.setCoefficients(driveHighSpeedPIDFCoefficients);
            distanceTolerance = DISTANCE_TOLERANCE_HIGH_SPEED;
            headingTolerance = HEADING_TOLERANCE_HIGH_SPEED;
            velocityTolerance = maxSpeed * drive.getMaxVelocity();

            headingPID.reset();
            drivePID.reset();
        } else {
            headingPID.setCoefficients(headingLowSpeedPIDFCoefficients);
            drivePID.setCoefficients(driveLowSpeedPIDFCoefficients);
            distanceTolerance = DISTANCE_TOLERANCE_LOW_SPEED;
            headingTolerance = HEADING_TOLERANCE_LOW_SPEED;
            velocityTolerance = VELOCITY_TOLERANCE_LOW_SPEED * drive.getMaxVelocity();
        }
    }

    /**
     * Move to the specified coordinate and heading
     */
    private void moveToCoordinate(boolean highSpeed) {

        Logger.message("to %3.0f, %3.0f, %4.0f", targetX, targetY, targetHeading);

        moveInit(highSpeed);
        timeoutTimer.reset();

        // Looping until we move the desired distance
        while (opMode.opModeIsActive() && !interruptAction) {
            double maxVelocity = drive.getMaxVelocity();
            Pose pose = getPose();
            double currentX = pose.getX();
            double currentY = pose.getY();
            double currentHeading = pose.getHeading();
            double a = targetY - currentY;
            double b = targetX - currentX;
            double signedAngle = polarToSignedAngle(currentHeading);
            double angle = Math.atan2(a, b) - signedAngle;   // angle is robot relative
            double distance = Math.hypot(a, b);
            double sin = Math.sin(angle - (Math.PI / 4));
            double cos = Math.cos(angle - (Math.PI / 4));
            double max = Math.max(Math.abs(sin), Math.abs(cos));
            double headingError = angleWrap(currentHeading - Math.toRadians(targetHeading));

            headingPID.updateError(headingError);
            double turn = headingPID.runPIDF();

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

            double leftFrontPower  = (power * (cos / max) + turn) / scale;
            double rightFrontPower = (power * (sin / max) - turn) / scale;
            double leftRearPower   = (power * (sin / max) + turn) / scale;
            double rightRearPower  = (power * (cos / max) - turn) / scale;

            drive.setVelocity(leftFrontPower * maxVelocity,
                              rightFrontPower * maxVelocity,
                              leftRearPower * maxVelocity,
                              rightRearPower * maxVelocity);

            double currentVelocity = drive.getCurrentVelocity();
            nearPose = currentVelocity / maxVelocity <= 0.20 && power < 0.5 && turn < 0.4;

            Logger.verbose("%s",
                    String.format("x: %-5.1f  y: %-5.1f  h: %-5.1f  ", currentX, currentY, Math.toDegrees(currentHeading)) +
                    String.format("a: %5.1f  b: %5.1f  distance: %5.2f  ", a, b, distance) +
                    String.format("heading error: %6.1f  ", Math.toDegrees(headingError)) +
                    String.format("angle: %4.0f  ", Math.toDegrees(angle)) +
                    String.format("turn: %5.2f  power: %4.2f  sin: %5.2f  cos: %5.2f  ", turn, power, sin, cos) +
                    String.format("wheels: %5.2f  %5.2f  %5.2f  %5.2f   ", leftFrontPower, rightFrontPower, leftRearPower, rightRearPower) +
                    String.format("velocity: %6.1f   ", currentVelocity) +
                    String.format("near: %5b  ", nearPose) +
                    String.format("volts: %4.1f  ", voltageSensor.getVoltage()) +
                    String.format("time: %4.0f   ", timeoutTimer.milliseconds())
            );

            if (Math.abs(a) < distanceTolerance && Math.abs(b) < distanceTolerance &&
                    Math.abs(Math.toDegrees(headingError)) < headingTolerance &&
                    Math.abs(currentVelocity) <= velocityTolerance) {
                if (this.highSpeed) {
                    Logger.message("low speed");
                    moveInit(false);
                } else {
                    break;
                }
            }

            if (timeout > 0 && timeoutTimer.milliseconds() >= timeout) {
                Logger.message("moveDistance timed out");
                break;
            }

            if (isEmergencyStop()) {
                break;
            }

        }

        drive.stopRobot();

        Pose pose = getPose();
        double x = pose.getX();
        double y =  pose.getY();
        double heading = Math.toDegrees(pose.getHeading());

        Logger.message("%s   %s   %s   %s",
                String.format("to: x %3.0f y %3.0f heading %4.0f", targetX, targetY, targetHeading),
                String.format("pose: x %5.1f  y %5.1f  heading %5.1f", x, y, heading),
                String.format("error: x %5.1f  y %5.1f  heading %5.1f", Math.abs(targetX-x), Math.abs(targetY-y), Math.abs(targetHeading-heading)),
                String.format("time: %4.2f", timeoutTimer.seconds()));
    }

    /**
     * Move to the specified pose.
     *
     * @param target target pose
     */
    private void moveToPose(Pose target) {

        Logger.message("target  x: %3.0f, y: %3.0f, heading: %4.0f  p: %6.3f",
                target.getX(), target.getY(), target.getHeading(AngleUnit.DEGREES), drivePID.getCoefficients().P);

        double maxVelocity = drive.getMaxVelocity();

        // Looping until we move the desired pose
        while (opMode.opModeIsActive() && !interruptAction) {

            Pose current = getPose();
            double a = target.getX() - current.getX();
            double b = target.getY() - current.getY();
            double distance = Math.hypot(a, b);
            double angle = Math.atan2(b, a);
            double sin = Math.sin(angle + (Math.PI / 4));
            double cos = Math.cos(angle + (Math.PI / 4));
            double headingError = AngleUnit.normalizeRadians(current.getHeading() - target.getHeading());
            double magnitude  = Math.hypot(localizer.getVelocityX(), localizer.getVelocityY());
            double velocityAngle = Math.atan2(localizer.getVelocityY(), localizer.getVelocityX());
            double deltaAngle = AngleUnit.normalizeRadians(angle - velocityAngle);
            double rotation = localizer.getVelocityHeading();

            double error = distance;
            if (magnitude > 10 && Math.abs(deltaAngle) < Math.PI/6) {
                error = Math.max(error - ((magnitude) * VELOCITY_COEFFICIENTS), 0);
            }

            drivePID.updateError(error);
            double power = drivePID.runPIDF();

            power = 0;

            headingPID.updateError(headingError);
            double turn = headingPID.runPIDF();

            // If the heading error is greater than 45 degrees then give the heading error greater weight
            if (Math.abs(headingError) < Math.toRadians(45))  {
                turn *= 2;
            }

            double scale = 1;
            if (power + Math.abs(turn) > maxSpeed) {
                scale = (power + Math.abs(turn)) / maxSpeed;
            }
            double max = Math.max(Math.abs(sin), Math.abs(cos));
            double leftFrontPower  = (power * (cos / max) + turn) / scale;
            double rightFrontPower = (power * (sin / max) - turn) / scale;
            double leftRearPower   = (power * (sin / max) + turn) / scale;
            double rightRearPower  = (power * (cos / max) - turn) / scale;

            drive.setVelocity(
                    leftFrontPower * maxVelocity,
                    rightFrontPower * maxVelocity,
                    leftRearPower * maxVelocity,
                    rightRearPower * maxVelocity);

            nearPose = magnitude <= 10 && Math.abs(rotation) < 10;

            Logger.verbose("%s",
                    String.format("x %5.1f  y %5.1f  h %5.1f  ", current.getX(), current.getY(), current.getHeading(AngleUnit.DEGREES)) +
                    String.format("distance: %5.2f  %5.2f  heading: %6.1f  ", distance, error, Math.toDegrees(headingError)) +
                    String.format("a: %5.1f  b: %5.1f  angle: %4.0f  sin: %5.2f  cos: %5.2f  ", a, b, Math.toDegrees(angle), sin, cos) +
                    String.format("turn: %5.2f  power: %4.2f  ", turn, power) +
                    String.format("wheels: %5.2f  %5.2f  %5.2f  %5.2f   ", leftFrontPower, rightFrontPower, leftRearPower, rightRearPower) +
                    String.format("vm: %3.0f  va: %4.0f  vh: %4.0f   ", magnitude, Math.toDegrees(deltaAngle), Math.toDegrees(rotation)) +
                    //String.format("velocity: %6.1f   ", currentVelocity) +
                    //String.format("near: %5b  ", nearPose) +
                    //String.format("volts: %4.1f  ", voltageSensor.getVoltage()) +
                    String.format("time: %4.0f   ", timeoutTimer.milliseconds())
            );

            if (Math.abs(a) < distanceTolerance && Math.abs(b) < distanceTolerance &&
                    Math.abs(Math.toDegrees(headingError)) < headingTolerance &&
                    Math.abs(magnitude) <= 5) {
                break;
            }

            if (timeout > 0 && timeoutTimer.milliseconds() >= timeout) {
                Logger.message("moveDistance timed out");
                break;
            }
        }
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

        headingPID.updateError(headingError);
        double turn = headingPID.runPIDF();

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

    private void moveToObject() {

        moveInit(true);
        timeoutTimer.reset();

        // Looping until we move the desired distance
        while (opMode.opModeIsActive() && !interruptAction) {

            double maxVelocity = drive.getMaxVelocity();
            Pose pose = getPose();

            double currentX = pose.getX();
            double currentY = pose.getY();
            double currentHeading = pose.getHeading();
            double distance = drive.distanceToObject() - stopDistance;
            double headingError = angleWrap(currentHeading - Math.toRadians(targetHeading));

            headingPID.updateError(headingError);
            double turn = headingPID.runPIDF();

            drivePID.updateError(distance);
            double power = drivePID.runPIDF();

            double scale = 1;
            if (power + Math.abs(turn) > maxSpeed)
                scale = (power + Math.abs(turn)) / maxSpeed;

            double leftFrontPower  = (power + turn) / scale;
            double rightFrontPower = (power - turn) / scale;
            double leftRearPower   = (power + turn) / scale;
            double rightRearPower  = (power - turn) / scale;

            drive.setVelocity(
                    leftFrontPower * maxVelocity,
                    rightFrontPower * maxVelocity,
                    leftRearPower * maxVelocity,
                    rightRearPower * maxVelocity);

            double currentVelocity = drive.getCurrentVelocity();

            if (verbose) {
                Logger.verbose("%s",
                    String.format("x: %-5.1f  y: %-5.1f  h: %-5.1f  ", currentX, currentY, Math.toDegrees(currentHeading)) +
                    String.format("distance: %5.2f  ", distance) +
                    String.format("heading error: %6.1f  ", Math.toDegrees(headingError)) +
                    String.format("turn: %5.2f  power: %4.2f  ", turn, power) +
                    String.format("power: %5.2f  %5.2f  %5.2f  %5.2f   ", leftFrontPower, rightFrontPower, leftRearPower, rightRearPower) +
                    String.format("velocity: %6.1f   ", currentVelocity) +
                    String.format("time: %4.0f", timeoutTimer.milliseconds())
                );
            }

            if (Math.abs(distance) < distanceTolerance &&
                    Math.abs(Math.toDegrees(headingError)) < headingTolerance &&
                    Math.abs(currentVelocity) <= velocityTolerance) {
                if (highSpeed) {
                    Logger.message("low speed");
                    moveInit(false);
                } else {
                    break;
                }
            }

            if (timeout > 0 && timeoutTimer.milliseconds() >= timeout) {
                Logger.message("moveDistance timed out");
                break;
            }

            if (isEmergencyStop()) {
                break;
            }
        }

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

    private void turnTo() {

        Logger.message("to %3.0f, %3.0f, %4.0f", targetX, targetY, targetHeading);

        timeoutTimer.reset();

        // Looping until we move the desired distance
        while (opMode.opModeIsActive() && !interruptAction) {

            double maxVelocity = drive.getMaxVelocity() * MAX_SPEED;
            Pose pose = getPose();
            double currentHeading = pose.getHeading();
            double signedAngle = polarToSignedAngle(currentHeading);
            double headingError = angleWrap(currentHeading - Math.toRadians(targetHeading));

            headingPID.updateError(headingError);
            double turn = headingPID.runPIDF();

            drive.setVelocity(
                      turn * maxVelocity,
                    -turn * maxVelocity,
                      turn * maxVelocity,
                    -turn * maxVelocity);

            double currentVelocity = drive.getCurrentVelocity();

            Logger.verbose("%s",
                    String.format("h: %-5.1f  ", Math.toDegrees(currentHeading)) +
                            String.format("heading error: %6.1f  ", Math.toDegrees(headingError)) +
                            String.format("signed angle: %4.0f  ", Math.toDegrees(signedAngle)) +
                            String.format("turn: %5.2f    ", turn) +
                            String.format("velocity: %6.1f   ", currentVelocity) +
                            String.format("time: %4.0f", timeoutTimer.milliseconds())
            );

            if (Math.abs(Math.toDegrees(headingError)) < headingTolerance &&
                    Math.abs(currentVelocity) <= velocityTolerance) {
                break;
            }

            if (timeout > 0 && timeoutTimer.milliseconds() >= timeout) {
                Logger.message("timed out");
                break;
            }

            if (isEmergencyStop()) {
                break;
            }
        }

        Logger.message("time: %4.2f", timeoutTimer.seconds());
        drive.stopRobot();

    }

    /**
     * Returns an signed angle in radians between -pi and pi.
     *
     * @param radians angle in radians
     * @return signed angle in radians between -pi and pi.
     */
    private double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }

    /**
     * Convert a polar angle, (0 - 2PI) where 0 is positive x to a signed angle (-PI to PI) where
     * 0 is positive y.
     *
     * @param polarAngle (0 - 2PI) 0 is positive x
     * @return signed angle (-PI to PI) 0 is positive y, counterclockwise is positive direction
     */
    private double polarToSignedAngle(double polarAngle) {

        double angle = polarAngle - Math.PI/2;
        if (angle > Math.PI)
            angle = angle - (Math.PI * 2);
        return angle;
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

    private boolean isEmergencyStop() {
        return opMode.gamepad1.back;        // todo remove
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

        moveToCoordinate(x, y, Math.toDegrees(pose.getHeading()), 2000);
        Logger.message("distance x: %6.1f  y: %6.1f   from x: %5.1f  y: %5.1f    to x: %5.1f  y: %5.1f ", distanceX, distanceY,pose.getX(), pose.getY(), x, y);
    }

    public void emergencyStop() {
        synchronized (this) {
            interruptAction();
            drive.stopRobot();
            driveState = DRIVE_STATE.IDLE;
        }
    }

    public void moveToCoordinate(double targetX, double targetY, double targetHeading, double maxSpeed, double timeout) {
        Logger.message("sync wait");
        synchronized (this) {
            interruptAction();

            this.targetX = targetX;
            this.targetY = targetY;
            this.targetHeading = targetHeading;
            this.maxSpeed = maxSpeed;
            this.minSpeed = MIN_SPEED;
            this.timeout = timeout;
            driveState = DRIVE_STATE.MOVING_TO_COORDINATE;
        }
        Logger.message("sync done");
    }

    public void moveToCoordinate(double targetX, double targetY, double targetHeading, double timeout) {
        moveToCoordinate(targetX, targetY, targetHeading, MAX_SPEED, timeout);
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

            Pose pose = getPose();
            double heading = pose.getHeading() + Math.toRadians(degrees);
            heading = Math.toDegrees(Pose.normalizeAngle(heading));

            this.targetX = pose.getX();
            this.targetY = pose.getY();
            this.targetHeading = heading;
            this.maxSpeed = MAX_TURN_SPEED;
            this.minSpeed = MIN_TURN_SPEED;
            this.timeout = timeout;
            driveState = DRIVE_STATE.TURN_TO;
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
            this.targetHeading = getPose().getHeading();
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
        Pose pose;
        localizer.update();
        pose = localizer.getPose();
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

    public Pose getVelocity() {
        localizer.update();
        return (localizer.getVelocity());
    }

    public void resetIMU() {
        localizer.resetIMU();
    }
}



