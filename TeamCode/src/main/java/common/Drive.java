/*
 * This this contains the class that controls the robot's drive train
 */

package common;

import android.annotation.SuppressLint;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@com.acmerobotics.dashboard.config.Config

@SuppressLint("DefaultLocale")
@SuppressWarnings("FieldCanBeLocal")
//@SuppressWarnings("unused")

public class Drive extends Thread {

    final boolean LOG_VERBOSE = false;

    public static double DRIFT_COEFFICIENT = 0.0015;

    // Drive train
    private final double MOTOR_TICKS_PER_REV = 384.5;           // Gobilda Yellow Jacket Motor 5203-2402-0014
    private final double MOTOR_RPM = 435;                       // Gobilda Yellow Jacket Motor 5203-2402-0014
    private final double MAX_VELOCITY = MOTOR_TICKS_PER_REV * MOTOR_RPM / 60;

    private final double WHEEL_DIAMETER_INCHES = (96 / 25.4);   // 96 mm wheels converted to inches
    private final double COUNTS_PER_INCH = MOTOR_TICKS_PER_REV / (WHEEL_DIAMETER_INCHES * Math.PI);
    //private final double TICKS_PER_MOTOR_REV = 28 * 20;         // HD Hex Motor Encoder Ticks * gearing
    private final double ODOMETER_COUNTS_PER_INCH = 2000/((48/25.4)*Math.PI); // counts per rev / wheel diameter in inches * pi

    private final double RAMP_DISTANCE = COUNTS_PER_INCH * 20;   // ramp down distance in encoder counts
    private final double RAMP_TIME = 1000;                       // ramp up time in milliseconds
    private final double RAMP_MIN_SPEED = 0.2;

    public static double MIN_SPEED = 0.20;
    public static double MAX_SPEED = 0.9;
    public static double MIN_STRAFE_SPEED = 0.35;
    public static double MAX_STRAFE_SPEED = 0.95;
    public static double MIN_ROTATE_SPEED = 0.15;
    public static double MAX_ROTATE_SPEED = 0.50;

    final double TURN_MIN_SPEED         = 0.2;

    final double TURN_RAMP_UP_TIME      = 1000;                       // turn ramp up time in milliseconds
    final double TURN_RAMP_DOWN_DEGREES = 5;



    public enum DIRECTION { FORWARD, BACK, LEFT, RIGHT, TURN_LEFT, TURN_RIGHT, DRIVER, STOOPED }

    // Color sensor
    static final float COLOR_SENSOR_GAIN = 2.2F;

    public enum COLOR {RED, BLUE}

    //  Drive train motors
    public DcMotorEx leftFrontDrive = null;   //  Used to control the left front drive wheel
    public DcMotorEx rightFrontDrive = null;  //  Used to control the right front drive wheel
    public DcMotorEx leftBackDrive = null;    //  Used to control the left back drive wheel
    public DcMotorEx rightBackDrive = null;   //  Used to control the right back drive wheel

    private Gyro gyro;
    public double yaw = 0;

    private final NormalizedColorSensor colorSensor = null;

    public DistanceSensor distanceSensor;

    private final ElapsedTime elapsedTime = new ElapsedTime();

    private boolean running = true;

    private int leftFrontStartPos  = 0;
    private int rightFrontStartPos = 0;
    private int leftBackStartPos   = 0;
    private int rightBackStartPos  = 0;

    ElapsedTime accelerationTime = new ElapsedTime();
    double accelerationLastSpeed;
    double accelerationLastTime;

    private DIRECTION lastDirection = DIRECTION.STOOPED;

    public List<DcMotorEx> motors;
    LinearOpMode opMode;

    /**
     * Contractor
     *
     * @param opMode instance of opMode
     */
    public Drive(LinearOpMode opMode) {
        this.opMode = opMode;
        this.setName("Drive");
        init();
    }

    /**
     * Initialize the drive train motors.
     */
    private void init() {

        try {
            gyro = new Gyro(opMode.hardwareMap, Config.IMU);
        } catch (Exception e) {
            Logger.error(e, "Hardware not found");
        }

        try {
            leftFrontDrive = opMode.hardwareMap.get(DcMotorEx.class, Config.LEFT_FRONT);
            rightFrontDrive = opMode.hardwareMap.get(DcMotorEx.class, Config.RIGHT_FRONT);
            leftBackDrive = opMode.hardwareMap.get(DcMotorEx.class, Config.LEFT_BACK);
            rightBackDrive = opMode.hardwareMap.get(DcMotorEx.class, Config.RIGHT_BACK);

            leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

            motors = Arrays.asList(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive);

            setBraking(true);
            setRunWithEncoders(false);

        } catch (Exception e) {
            Logger.error(e, "Drive hardware not found");
        }

        try {
            distanceSensor = opMode.hardwareMap.get(DistanceSensor.class, Config.DISTANCE_SENSOR);
        } catch (Exception e) {
            Logger.error(e, "Distance sensor not found");
        }

        /*
        try {
            colorSensor = opMode.hardwareMap.get(NormalizedColorSensor.class, Config.COLOR_SENSOR);
            colorSensor.setGain(COLOR_SENSOR_GAIN);
        } catch (Exception e) {
            Logger.error(e, "Color sensor not found");
        }
         */
    }

    /**
     * Driver controlled robot movement runs on a separate thread
     */
    public void run() {

       Logger.message("robot drive thread started");

        while (running) {
            Thread.yield();
        }
       Logger.message("robot drive thread stopped");
    }

    /**
     * Stop the thread's run method
     */
    public void end () {
        running = false;
    }

    public double getMinPower () {
        return MIN_SPEED;
    }

    public double getMaxPower ( ) {
        return MAX_SPEED;
    }

    public double getMinTurnPower () {

        return MIN_ROTATE_SPEED;
    }

    public double getMaxTurnPower ( ) {
        return MAX_ROTATE_SPEED;
    }

    public double getMaxVelocity () {
        return MAX_VELOCITY;
    }

    public void  setVelocity (double velocity) {
        leftFrontDrive.setVelocity(velocity);
        rightFrontDrive.setVelocity(velocity);
        leftBackDrive.setVelocity(velocity);
        rightBackDrive.setVelocity(velocity);
    }

    public void setVelocity (double leftFront, double rightFront, double leftRear, double rightRear) {
        leftFrontDrive.setVelocity(leftFront);
        rightFrontDrive.setVelocity(rightFront);
        leftBackDrive.setVelocity(leftRear);
        rightBackDrive.setVelocity(rightRear);
    }

    public double getCurrentVelocity() {
        return (Math.max(Math.abs(leftFrontDrive.getVelocity()),
                Math.max(Math.abs(rightFrontDrive.getVelocity()),
                        Math.max(Math.abs(leftBackDrive.getVelocity()),
                                Math.abs(rightBackDrive.getVelocity())))));
    }

    public void setBraking(boolean enabled) {

        for (DcMotor motor : motors) {
            if (enabled)
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            else
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void setRunWithEncoders(boolean runWithEncoders) {
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            if (runWithEncoders)
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            else
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void resetEncoders() {

        for (DcMotor motor : motors) {
            DcMotor.RunMode mode = motor.getMode();
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(mode);
        }
    }

    public double encoderTicksPerInch() {
        return (COUNTS_PER_INCH);
    }

    /**
     *  Return the current orientation of the robot.
     * @return orientation in degrees in a range of -180 to 180
     */
    public double getOrientation() {
        yaw = gyro.getYaw();
        return yaw;
    }

    public void resetOrientation() {
        gyro.resetYaw();
    }

    /**
     * Drive the robot with gamepad joysticks and trigger
     */
    public void accelerationReset () {
        accelerationTime.reset();
        accelerationLastSpeed = 0;
        accelerationLastTime = 0;
    }

    /**
     * Limit the acceleration of the drivetrain.
     *
     * @param speed desired speed
     * @return limited speed
     */
    public double accelerationLimit(double speed) {

        double ACCELERATION_TIME = (1000 * 1.0);   // 1.5 second to accelerate to full speed
        double DECELERATION_TIME = (1000 * 1.0);   // 1 second to come to full stop

        double accelerationPerMS = (MAX_SPEED - MIN_SPEED) / ACCELERATION_TIME;
        double decelerationPerMS = (MAX_SPEED - MIN_SPEED) / DECELERATION_TIME;

        double currentTime = accelerationTime.milliseconds();
        double deltaSpeed = speed - accelerationLastSpeed;
        double deltaTime = currentTime - accelerationLastTime;
        double acceleration = deltaSpeed / deltaTime;

        if ((deltaSpeed > 0) && acceleration > accelerationPerMS)
            return accelerationLastSpeed + (accelerationPerMS * deltaTime);

        if ((deltaSpeed < 0) && (Math.abs(acceleration) > (decelerationPerMS * deltaTime)))
            return  accelerationLastSpeed - (decelerationPerMS * deltaTime);

        accelerationLastSpeed = speed;
        accelerationLastTime = currentTime;

        return speed;
    }

    public double distanceToObject () {
        if (distanceSensor != null)
            return distanceSensor.getDistance(DistanceUnit.INCH);

        return -1;
    }

    /**
     * Move robot according to desired axes motions
     *
     * @param x   Positive is forward
     * @param y   Positive is strafe left
     * @param yaw Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {

        double leftFrontPower  = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower   = x + y - yaw;
        double rightBackPower  = x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        if (LOG_VERBOSE) {
            Logger.message("power %f %f %f %f", leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        }
    }

    public void moveRobot(double x, double y, double yaw, double speed) {

        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;

        if (x == 0 && y == 0 && yaw == 0 ) {
            leftFrontPower = 0;
            rightFrontPower = 0;
            leftBackPower = 0;
            rightBackPower = 0;

        } else {
            leftFrontPower = x - y - yaw;
            rightFrontPower = x + y + yaw;
            leftBackPower = x + y - yaw;
            rightBackPower = x - y + yaw;

            // Normalize wheel powers to be less than 1.0
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (speed == 0)
                speed = MIN_SPEED;
            else if (speed > MAX_SPEED) {
                speed = MAX_SPEED;
            }
            if (yaw != 0 && (x == 0 && y == 0)) {
                if (speed > MAX_ROTATE_SPEED)
                    speed = MAX_ROTATE_SPEED;
            }

            double scale = (1 / max) * speed;

            leftFrontPower *= scale;
            rightFrontPower *= scale;
            leftBackPower *= scale;
            rightBackPower *= scale;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        if (LOG_VERBOSE) {
            Logger.message("power %f %f %f %f %f", speed, leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        }
    }

    /**
     *  Method to move the robot in the specified direction. The encoder are used the correct for drift
     *
     * @param direction direction to move
     * @param speed motor speed (-1 to 1)
     */
    public void moveRobot(DIRECTION direction, double speed) {

        int leftFrontSign = 0;
        int rightFrontSign = 0;
        int leftBackSign = 0;
        int rightBackSign = 0;

        if (direction == DIRECTION.FORWARD) {
            leftFrontSign  = 1;
            rightFrontSign = 1;
            leftBackSign   = 1;
            rightBackSign  = 1;
        } else if (direction == DIRECTION.BACK) {
            leftFrontSign  = -1;
            rightFrontSign = -1;
            leftBackSign   = -1;
            rightBackSign  = -1;
        } else if (direction == DIRECTION.LEFT) {
            leftFrontSign  = -1;
            rightFrontSign =  1;
            leftBackSign   =  1;
            rightBackSign  = -1;
        } else if (direction == DIRECTION.RIGHT) {
            leftFrontSign  =  1;
            rightFrontSign = -1;
            leftBackSign   = -1;
            rightBackSign  =  1;
        } else if (direction == DIRECTION.TURN_LEFT) {
            leftFrontSign  = -1;
            rightFrontSign =  1;
            leftBackSign   = -1;
            rightBackSign  =  1;
        } else if (direction == DIRECTION.TURN_RIGHT){
            leftFrontSign  =  1;
            rightFrontSign = -1;
            leftBackSign   =  1;
            rightBackSign  = -1;
        }

        // If the direction has changed get the encoder positions.
        if (direction != lastDirection) {
            leftFrontStartPos = leftFrontDrive.getCurrentPosition();
            rightFrontStartPos = rightFrontDrive.getCurrentPosition();
            leftBackStartPos = leftBackDrive.getCurrentPosition();
            rightBackStartPos = rightBackDrive.getCurrentPosition();
            lastDirection = direction;
            accelerationReset();
        }

        speed = accelerationLimit(speed);

        // Correct for drift
        double leftFrontPos =  Math.abs(leftFrontDrive.getCurrentPosition()  - leftFrontStartPos);
        double rightFrontPos = Math.abs(rightFrontDrive.getCurrentPosition() - rightFrontStartPos);
        double leftBackPos =   Math.abs(leftBackDrive.getCurrentPosition()   - leftBackStartPos);
        double rightBackPos =  Math.abs(rightBackDrive.getCurrentPosition()  - rightBackStartPos);
        double maxPos = Math.max(Math.max(Math.max(leftFrontPos, rightFrontPos), leftBackPos), rightBackPos);

        double leftFrontAdjust = (maxPos - leftFrontPos) * DRIFT_COEFFICIENT;
        double rightFrontAdjust = (maxPos - rightFrontPos) * DRIFT_COEFFICIENT;
        double leftBackAdjust = (maxPos - leftBackPos) * DRIFT_COEFFICIENT;
        double rightBackAdjust = (maxPos - rightBackPos) * DRIFT_COEFFICIENT;

        double leftFrontPower = (speed + leftFrontAdjust) * leftFrontSign;
        double rightFrontPower = (speed + rightFrontAdjust) * rightFrontSign;
        double leftBackPower = (speed + leftBackAdjust) * leftBackSign;
        double rightBackPower = (speed + rightBackAdjust) * rightBackSign;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        if (LOG_VERBOSE) {
            Logger.message("moveDistance: power: %4.2f %4.2f %4.2f %4.2f    adjust: %4.3f %4.3f %4.3f %4.3f     position: %6.0f %6.0f %6.0f %6.0f",
                    leftFrontPower,
                    rightFrontPower,
                    leftBackPower,
                    rightBackPower,
                    leftFrontAdjust,
                    rightFrontAdjust,
                    leftBackAdjust,
                    rightBackAdjust,
                    leftFrontPos,
                    rightFrontPos,
                    leftBackPos,
                    rightBackPos);
        }
    }


    /**
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     *
     * @param direction direction to move
     * @param speed motor speed (-1 to 1)
     * @param inches  distance to move in inches, positive for forward, negative for backward
     * @param timeout timeout in milliseconds, 0 for no timeout
     */
    public void moveDistance(DIRECTION direction, double speed, double inches, double timeout) {

        int leftFrontSign = 0;
        int rightFrontSign = 0;
        int leftBackSign = 0;
        int rightBackSign = 0;

        if (direction == DIRECTION.FORWARD) {
            leftFrontSign  = 1;
            rightFrontSign = 1;
            leftBackSign   = 1;
            rightBackSign  = 1;
        } else if (direction == DIRECTION.BACK) {
            leftFrontSign  = -1;
            rightFrontSign = -1;
            leftBackSign   = -1;
            rightBackSign  = -1;
        } else if (direction == DIRECTION.LEFT) {
            leftFrontSign  = -1;
            rightFrontSign =  1;
            leftBackSign   =  1;
            rightBackSign  = -1;
        } else if (direction == DIRECTION.RIGHT) {
            leftFrontSign  =  1;
            rightFrontSign = -1;
            leftBackSign   = -1;
            rightBackSign  =  1;
        } else if (direction == DIRECTION.TURN_LEFT) {
            leftFrontSign  = -1;
            rightFrontSign =  1;
            leftBackSign   = -1;
            rightBackSign  =  1;
        } else if (direction == DIRECTION.TURN_RIGHT){
            leftFrontSign  =  1;
            rightFrontSign = -1;
            leftBackSign   =  1;
            rightBackSign  = -1;
        }

        DcMotor.RunMode mode = leftFrontDrive.getMode();
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Determine new target position, and pass to motor controller
        int target = (int) (inches * encoderTicksPerInch());
        leftFrontDrive.setTargetPosition(target * leftFrontSign);
        rightFrontDrive.setTargetPosition(target * rightFrontSign);
        leftBackDrive.setTargetPosition(target * leftBackSign);
        rightBackDrive.setTargetPosition(target * rightBackSign);

        // Turn On RUN_TO_POSITION
        for (DcMotor motor : motors)
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Looping until we move the desired distance
        elapsedTime.reset();
        boolean moving = true;
        while (opMode.opModeIsActive() && moving) {

            // Correct for drift
            double leftFrontPos = Math.abs(leftFrontDrive.getCurrentPosition());
            double rightFrontPos = Math.abs(rightFrontDrive.getCurrentPosition());
            double leftBackPos = Math.abs(leftBackDrive.getCurrentPosition());
            double rightBackPos = Math.abs(rightBackDrive.getCurrentPosition());
            double maxPos = Math.max(Math.max(Math.max(leftFrontPos, rightFrontPos), leftBackPos), rightBackPos);

            double speedRange = Math.max(Math.abs(speed) - RAMP_MIN_SPEED, 0);
            double ramUp = (elapsedTime.milliseconds() / RAMP_TIME) * speedRange + RAMP_MIN_SPEED;
            double ramDown = (Math.pow((Math.abs(target) - maxPos), 2) / Math.pow(RAMP_DISTANCE, 2)) * speedRange + RAMP_MIN_SPEED;
            double rampPower = Math.min(Math.min(ramUp, ramDown), speed);

            double leftFrontAdjust = (maxPos - leftFrontPos) * DRIFT_COEFFICIENT;
            double rightFrontAdjust = (maxPos - rightFrontPos) * DRIFT_COEFFICIENT;
            double leftBackAdjust = (maxPos - leftBackPos) * DRIFT_COEFFICIENT;
            double rightBackAdjust = (maxPos - rightBackPos) * DRIFT_COEFFICIENT;

            leftFrontDrive.setPower((rampPower + leftFrontAdjust) * leftFrontSign);
            rightFrontDrive.setPower((rampPower + rightFrontAdjust) * rightFrontSign);
            leftBackDrive.setPower((rampPower + leftBackAdjust) * leftBackSign);
            rightBackDrive.setPower((rampPower + rightBackAdjust) * rightBackSign);

            for (DcMotor motor : motors)
                if (! motor.isBusy())
                    moving = false;

            if (timeout > 0 && elapsedTime.milliseconds() >= timeout) {
                Logger.message("moveDistance: timed out");
                break;
            }

            if (LOG_VERBOSE) {
                Logger.message("power: %4.2f %4.2f %4.2f %4.2f   %5.2f %5.2f %5.2f    remaining: %5.2f    adjust: %4.2f %4.2f %4.2f %4.2f     position: %6d %6d %6d %6d     velocity: %4.0f %4.0f %4.0f %4.0f    heading %6.1f ",
                        leftFrontDrive.getPower(),
                        rightFrontDrive.getPower(),
                        leftBackDrive.getPower(),
                        rightBackDrive.getPower(),
                        rampPower, ramUp, ramDown,
                        (Math.abs(target) - maxPos) / COUNTS_PER_INCH,
                        leftFrontAdjust,
                        rightFrontAdjust,
                        leftBackAdjust,
                        rightBackAdjust,
                        leftFrontDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition(),
                        leftBackDrive.getCurrentPosition(),
                        rightBackDrive.getCurrentPosition(),
                        leftFrontDrive.getVelocity(),
                        rightFrontDrive.getVelocity(),
                        leftBackDrive.getVelocity(),
                        rightBackDrive.getVelocity(),
                        getOrientation());
            }
        }

        // Stop all motion;
        for (DcMotor motor : motors)
            motor.setPower(0);

        // Restore run mode to prior state
        for (DcMotor motor : motors)
            motor.setMode(mode);

        Logger.message("%s  target  %6.2f  traveled %6.2f %6.2f %6.2f %6.2f  heading %6.2f  time %6.2f",
                direction,
                (double)target / encoderTicksPerInch(),
                (double)leftFrontDrive.getCurrentPosition() / encoderTicksPerInch(),
                (double)rightFrontDrive.getCurrentPosition() / encoderTicksPerInch(),
                (double)leftBackDrive.getCurrentPosition() / encoderTicksPerInch(),
                (double)rightBackDrive.getCurrentPosition() / encoderTicksPerInch(),
                getOrientation(),
                elapsedTime.seconds());
    }

    /**
     * Move the robot until the specified color is detected.
     *
     * @param color the color to detect
     * @param x positive for forward, negative for backwards
     * @param y positive for strafe left ???, negative for strafe right ???
     * @param speed drive speed
     * @param maxInches maximum distance to travel in inches, (0 for no maximum)
     * @param timeout timeout in milliseconds (0 for no timeout)
     */
    public boolean moveToColor(COLOR color, double x, double y, double speed, double maxInches, double timeout){

        boolean found = false;
        float[] hsvValues = new float[3];
        ElapsedTime elapsedTime = new ElapsedTime();
        float lastHue = 0;
        float lastSaturation = 0;

        resetEncoders();
        moveRobot(x, y, 0, speed);
        elapsedTime.reset();

        while (! found && opMode.opModeIsActive())  {
            // Get the normalized colors from the sensor
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            // Convert to HSV color space
            Color.colorToHSV(colors.toColor(), hsvValues);
            float hue = hsvValues[0];
            float saturation = hsvValues[1];

            if (hue != lastHue || saturation != lastSaturation) {
                Logger.message("hue %f saturation %f", hue, saturation);
                lastHue = hue;
                lastSaturation = saturation;
            }
            if (color == COLOR.BLUE) {
                if (hue >= 190 && hue <= 230 && saturation >= .5) {
                    Logger.message("blue line found");
                    found = true;
                }
            } else if (color == COLOR.RED) {
                if ((hue >= 0 && hue <= 90) && saturation >= .5) {
                    Logger.message("red line found");
                    found = true;
                }
            }
            if (maxInches > 0 && (maxInches <= getDistanceTraveled())) {
                Logger.warning("no line found, traveled %5.2f inches", getDistanceTraveled());
                break;
            }
            if (timeout > 0 && elapsedTime.milliseconds() > timeout){
                Logger.warning("timeout, no line found, traveled %5.2f inches", getDistanceTraveled());
                break;
            }
        }
        stopRobot();
        return found;
    }

    /**
     * Stop all the drive train motors.
     */
    public void stopRobot() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        lastDirection = DIRECTION.STOOPED;
    }

    public List<Double> getWheelPositions() {

        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotor motor : motors) {
            int position = motor.getCurrentPosition();
            wheelPositions.add((double)position / encoderTicksPerInch());
        }
        return wheelPositions;
    }

    public double getDistanceTraveled() {
        double traveled = 0;

        for (DcMotor motor : motors)
            traveled += motor.getCurrentPosition();
        return traveled / motors.size() / encoderTicksPerInch();
    }

    public void forward (double distance) {
        moveDistance(DIRECTION.FORWARD,.4, distance, 0);
    }

    public void back (double distance) {

        moveDistance(DIRECTION.BACK,.4, distance, 0);
    }

    public void strafeLeft (double distance) {
        //distance = Math.sqrt(Math.pow(distance,2 ) + Math.pow(distance,2));
        distance *= Settings.getStrafeFactor();
        moveDistance(DIRECTION.LEFT,.4, distance, 0);
    }

    public void strafeRight (double distance) {
        //distance = Math.sqrt(Math.pow(distance,2 ) + Math.pow(distance,2));
        distance *= Settings.getStrafeFactor();
        moveDistance(DIRECTION.RIGHT,.4, distance, 0);
    }

    public void turn(double degrees) {

        double circumference = 2 * Math.PI * Settings.getTurnFactor();
        double inches = Math.abs(degrees) / 360 * circumference;
        if (degrees > 0)
            moveDistance(DIRECTION.TURN_LEFT, 0.4,  inches, 0 );
        else
            moveDistance(DIRECTION.TURN_RIGHT, 0.4,  inches, 0 );
    }

    private double getRampedPower (double startTime, double speed, double degreesToGo) {

        double speedRange = Math.max(Math.abs(speed) - TURN_MIN_SPEED, 0);
        double ramUp = (startTime / TURN_RAMP_UP_TIME) * speedRange + TURN_MIN_SPEED;
        double ramDown = (Math.pow(degreesToGo, 2) / Math.pow(TURN_RAMP_DOWN_DEGREES, 2)) * speedRange + TURN_MIN_SPEED;
        return Math.min(Math.min(ramUp, ramDown), speed);
    }

    private double rampPower (double speed, double target) {

        int position = (
                Math.abs(leftFrontDrive.getCurrentPosition()) +
                        Math.abs(rightFrontDrive.getCurrentPosition()) +
                        Math.abs(leftBackDrive.getCurrentPosition()) +
                        Math.abs(rightBackDrive.getCurrentPosition()) ) / 4;

        double speedRange = Math.max(Math.abs(speed) - RAMP_MIN_SPEED, 0);
        double ramUp = (elapsedTime.milliseconds() / RAMP_TIME) * speedRange + RAMP_MIN_SPEED;
        double ramDown = ((Math.abs(target) - (double) position) / RAMP_DISTANCE) * speedRange + RAMP_MIN_SPEED;
        //double ramDown = (Math.pow((Math.abs(target) - position), 2) / Math.pow(RAMP_DISTANCE, 2)) * speedRange + RAMP_MIN_SPEED;
        double rampPower = Math.min(Math.min(ramUp, ramDown), speed);
/*
        Logger.message("target %6.2f  position %d ramUp %5.2f  ramDown %5.2f  ramPower %5.2f  %5.2f  ",
                target, position, ramUp, ramDown, rampPower,
                ((Math.abs(target) - position) / RAMP_DISTANCE)
                );
*/
        return rampPower;
    }


}

