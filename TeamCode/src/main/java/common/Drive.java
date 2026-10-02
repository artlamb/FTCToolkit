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
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Arrays;
import java.util.List;

//@com.acmerobotics.dashboard.config.Config

@SuppressLint("DefaultLocale")
@SuppressWarnings("FieldCanBeLocal")
//@SuppressWarnings("unused")

public class Drive {

    final boolean LOG_VERBOSE = false;

    public static double PID_P = 10;
    public static double PID_I = 3;

    // Drive train
    private final double MOTOR_TICKS_PER_REV = 384.5;           // Gobilda Yellow Jacket Motor 5203-2402-0014
    private final double MOTOR_RPM = 435;                       // Gobilda Yellow Jacket Motor 5203-2402-0014
    private final double MAX_VELOCITY = MOTOR_TICKS_PER_REV * MOTOR_RPM / 60;

    private final double WHEEL_DIAMETER_INCHES = (96 / 25.4);   // 96 mm wheels converted to inches
    private final double COUNTS_PER_INCH = MOTOR_TICKS_PER_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

    public double MIN_SPEED = 0.20;
    public double MAX_SPEED = 0.9;
    public double MIN_ROTATE_SPEED = 0.15;
    public double MAX_ROTATE_SPEED = 0.50;

    // Color sensor
    static final float COLOR_SENSOR_GAIN = 2.2F;

    public enum COLOR {RED, BLUE}

    //  Drive train motors
    public DcMotorEx leftFront = null;   //  Used to control the left front drive wheel
    public DcMotorEx rightFront = null;  //  Used to control the right front drive wheel
    public DcMotorEx leftRear = null;    //  Used to control the left back drive wheel
    public DcMotorEx rightRear = null;   //  Used to control the right back drive wheel

    private final NormalizedColorSensor colorSensor = null;

    ElapsedTime accelerationTime = new ElapsedTime();
    double accelerationLastSpeed;
    double accelerationLastTime;

    boolean braking;
    boolean runWithEncoders;
    PIDFCoefficients coefficients;

    public List<DcMotorEx> motors;
    LinearOpMode opMode;

    /**
     * Contractor
     *
     * @param opMode instance of opMode
     */
    public Drive(LinearOpMode opMode) {
        this.opMode = opMode;
        init();
    }

    /**
     * Initialize the drive train motors.
     */
    private void init() {

        try {
            leftFront = opMode.hardwareMap.get(DcMotorEx.class, Config.LEFT_FRONT);
            rightFront = opMode.hardwareMap.get(DcMotorEx.class, Config.RIGHT_FRONT);
            leftRear = opMode.hardwareMap.get(DcMotorEx.class, Config.LEFT_BACK);
            rightRear = opMode.hardwareMap.get(DcMotorEx.class, Config.RIGHT_BACK);

            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
            leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
            rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

            motors = Arrays.asList(leftFront, rightFront, leftRear, rightRear);

            setPIDFCoefficients(PID_P, PID_I);

            setBraking(true);
            setRunWithEncoders(false);

        } catch (Exception e) {
            Logger.error(e, "Drive hardware not found");
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
        leftFront.setVelocity(velocity);
        rightFront.setVelocity(velocity);
        leftRear.setVelocity(velocity);
        rightRear.setVelocity(velocity);
    }

    public void setVelocity (double leftFront, double rightFront, double leftRear, double rightRear) {
        this.leftFront.setVelocity(leftFront);
        this.rightFront.setVelocity(rightFront);
        this.leftRear.setVelocity(leftRear);
        this.rightRear.setVelocity(rightRear);
    }

    public double getCurrentVelocity() {
        return (Math.max(Math.abs(leftFront.getVelocity()),
                Math.max(Math.abs(rightFront.getVelocity()),
                        Math.max(Math.abs(leftRear.getVelocity()),
                                Math.abs(rightRear.getVelocity())))));
    }

    public void setPower (double power) {
        leftFront.setVelocity(power);
        rightFront.setVelocity(power);
        leftRear.setVelocity(power);
        rightRear.setVelocity(power);
    }

    public void setPIDFCoefficients(double p, double i) {
        coefficients = new PIDFCoefficients(p, 0, i, 0);
        for (DcMotorEx motor : motors) {
            PIDFCoefficients coefficients =  motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            coefficients.p = p;
            coefficients.i = i;
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
            Logger.message("drive pid coefficients: %s", coefficients.toString());
        }
    }

    public double getCurrent() {
        double current = 0;
        for (DcMotorEx motor : motors) {
            current += motor.getCurrent(CurrentUnit.AMPS);
        }
        return current;
    }

    public void setBraking(boolean enabled) {
        for (DcMotor motor : motors) {
            if (enabled)
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            else
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        braking = enabled;
    }

    public void setRunWithEncoders(boolean runWithEncoders) {
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            if (runWithEncoders)
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            else
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        this.runWithEncoders = runWithEncoders;
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
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftRear.setPower(leftBackPower);
        rightRear.setPower(rightBackPower);

        if (LOG_VERBOSE) {
            Logger.message("power %f %f %f %f %f", speed, leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        }
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

        if (colorSensor == null) {
            Logger.warning("color sensor not found");
            return false;
        }

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
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    public double getDistanceTraveled() {
        double traveled = 0;

        for (DcMotor motor : motors)
            traveled += motor.getCurrentPosition();
        return traveled / motors.size() / encoderTicksPerInch();
    }

    /**
     * Engages the RUN_TO_POSITION mode on all motors to hold the last set target.
     * The motors will be given a power value to actively hold their positions.
     */
    public void holdPosition() {

        PIDFCoefficients coefficients = new PIDFCoefficients(PID_P, PID_I, 0, 0);

        for (DcMotorEx motor : motors) {
            int position = motor.getCurrentPosition();
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setTargetPosition(position);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // Give the motors power to hold their position.
        // This power value acts as a *maximum speed limit* for the PID controller.
        // A value of 1.0 is common, but you can use a smaller value if you want
        // the corrections to be less aggressive.
        setPower(1.0);
    }

    /**
     * Releases the motors from holding position and returns to a standard drive mode.
     */
    public void releasePosition() {
        setPIDFCoefficients(PID_P, PID_I);
        setRunWithEncoders(runWithEncoders);
        setBraking(braking);
        setPower(0.0);
    }
}

