package competition;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import common.Config;
import common.DriveControl;
import common.Hopper;
import common.Intake;
import common.Launcher;
import common.Limelight;
import common.Logger;
import common.Robot;
import utils.Increment;
import utils.Pose;

@TeleOp(name="TeleOpComp", group="Competition")
@SuppressLint("DefaultLocale")
@com.acmerobotics.dashboard.config.Config

public class TeleOpComp extends LinearOpMode {

    public static boolean useOdometer = true;
    public static double DEFAULT_SPEED = 28;

    private Robot robot;
    private DriveControl driveControl;
    private Launcher launcher;
    private Limelight limelight;
    private Intake intake;
    private Hopper hopper;
    private LED redLeftLED;
    private LED redRightLED;
    private LED greenLeftLED;
    private LED greenRightLED;

    Increment speedIncrement;
    double speed = DEFAULT_SPEED;

    Telemetry.Item speedMsg;
    Telemetry.Item aprilTagMsg;

    long lastUpdate;
    double distance = 0;

    boolean customSpeed = false;

    long odometerSetTime = 0;
    int aprilTagID = 0;

    enum LEDColor { GREEN, RED, YELLOW, NONE }

    @Override
    public void runOpMode() {
        try {
            initialize();

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            telemetry.update();

            setLED(LEDColor.NONE);

            waitForStart();

            while (opModeIsActive()) {
                handleGamepad();
                updatePosition();
                updateCustomSpeed();
            }

        } catch (Exception e) {
            Logger.error(e, "Error");
        }
    }

    private void initialize() {

        robot = new Robot(this);
        robot.startDriveGamepad();

        driveControl = robot.getDriveControl();
        driveControl.reset();

        launcher = robot.getLauncher();
        launcher.setSpeed(speed);

        limelight = robot.getLimelight();
        limelight.setPipeline(Limelight.Pipeline.LOCATION);

        intake = robot.getIntake();
        hopper = robot.getHopper();

        greenLeftLED = hardwareMap.get(LED.class, Config.GREEN_LEFT_LED);
        greenRightLED = hardwareMap.get(LED.class, Config.GREEN_RIGHT_LED);
        redLeftLED = hardwareMap.get(LED.class, Config.RED_LEFT_LED);
        redRightLED = hardwareMap.get(LED.class, Config.RED_RIGHT_LED);

        speedIncrement = new Increment(1, 2, 3);

        aprilTagMsg = telemetry.addData("April Tag", "");

        speedMsg = telemetry.addData("Motor speed", 0);
        displaySpeed();

        telemetry.addData("\nControls", "\n" +
                "  a - start / stop launcher motors\n" +
                "  b - line up with april tag\n" +
                "  x - open / close close loader gate\n" +
                "  y - start / stop intake\n" +
                "  right trigger - fire artifact\n" +
                "  left trigger - fire all artifacts\n" +
                "  left bumper - decrease motor speed\n" +
                "  right bumper - increase motor speed\n" +
                "\n");
    }

    /**
     * Handle gamepad input
     */
    private void handleGamepad() {

        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            // start or stop the launcher motors
            robot.powerLauncher(! launcher.isRunning(), speed);

        } else if (gamepad1.yWasPressed() || gamepad2.yWasPressed()) {
            // turn the intake on or off and open, close the lever
            robot.powerIntake(! intake.isRunning());

        } else if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
            // line up with the goal
            lineUpWithGoal(false);
            setSpeed();

        } else if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) {
            // open /close the loader gate
            launcher.gateToggle();

        } else if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()){
            // pull the trigger
            launcher.pullTrigger();

        } else if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed()) {
            // line up with the goal and fire all artifacts
            hopper.leverToggle();

        } else if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed()) {
            // reverse the intake
            intake.toggleReverse();

        } else if (gamepad1.right_trigger > 0) {
            // fire one artifact
            fire();
            while (gamepad1.right_trigger > 0) {
                Thread.yield();
            }
        } else if (gamepad2.right_trigger > 0) {
            // fire one artifact
            fire();
            while (gamepad2.right_trigger > 0) {
                Thread.yield();
            }

        } else if (gamepad1.left_trigger > 0) {
            // fire all artifacts
            fireAll();
            while (gamepad1.left_trigger > 0) {
                Thread.yield();
            }
        } else if (gamepad2.left_trigger > 0) {
            // fire all artifacts
            fireAll();
            while (gamepad2.left_trigger > 0) {
                Thread.yield();
            }

        } else if (gamepad1.left_bumper) {
            // increase motor speed
            speedIncrement.reset();
            while (gamepad1.left_bumper) {
                speed = Math.max(speed - speedIncrement.get(), 0);
                displaySpeed();
                telemetry.update();
            }
            setCustomSpeed();
        } else if (gamepad2.left_bumper) {
            // increase motor speed
            speedIncrement.reset();
            while (gamepad2.left_bumper) {
                speed = Math.max(speed - speedIncrement.get(), 0);
                displaySpeed();
                telemetry.update();
            }
            setCustomSpeed();

        } else if (gamepad1.right_bumper) {
            // decrease the motor speed
            speedIncrement.reset();
            while (gamepad1.right_bumper) {
                speed = Math.min(speed + speedIncrement.get(), 140);
                displaySpeed();
                telemetry.update();
            }
            setCustomSpeed();
        } else if (gamepad2.right_bumper) {
            // decrease the motor speed
            speedIncrement.reset();
            while (gamepad2.right_bumper) {
                speed = Math.min(speed + speedIncrement.get(), 140);
                displaySpeed();
                telemetry.update();
            }
            setCustomSpeed();
        }
    }

    /**
     * Line up with the goal's april tag
     *
     * @param displayOnly if true, don't move the robot just display the position of the robot based on the goal's april tag
     */
    private void lineUpWithGoal(boolean displayOnly) {

        distance = 0;
        int blueID = 20;
        int redID = 24;
        Pose current = null;

        // Get the position of the robot based on the goal's april tag
        int id = limelight.GetAprilTagID();
        if (id == blueID || id == redID) {
            current = limelight.getPosition();
            if (current != null) {

                // set the odometer's position to the april tag's robot position every 20 seconds
                if (useOdometer) {
                    long time = System.currentTimeMillis();
                    if (odometerSetTime == 0 || time - odometerSetTime > 20000) {
                        odometerSetTime = time;
                        aprilTagID = id;
                        driveControl.setPose(current);
                        Logger.message("odometer's position set to: %s", current);
                    }
                }
                setLED(LEDColor.GREEN);
            }
        }

        // no april tag found
        if (current == null) {
            // If we can see the april tag and the odometer's position has been set, use it.
            if (useOdometer && odometerSetTime != 0) {
                current = driveControl.getPose();
                id = aprilTagID;
                Logger.message("no april tag found, using odometer's position: %s", current);
                setLEDs(LEDColor.GREEN, LEDColor.YELLOW);

            } else {
                displayAprilTagInfo("april tag if not found");
                setLED(LEDColor.RED);
                return;
            }
        }


        // aim for the center of the goal
        Pose target;
        double num = 70.5 - 6;  // field quadrant size minus 6 inches
        if (id == blueID) {
            target = new Pose(-num, num, Math.toRadians(135));
        } else {   //red
            target = new Pose(num, num, Math.toRadians(45));
        }

        double a = target.getX() - current.getX();
        double b = target.getY() - current.getY();
        double angle = Math.atan2(b, a);
        double rotation = AngleUnit.normalizeRadians(current.getHeading() - angle);

        distance = Math.abs(Math.hypot(a, b));
        double velocity = getVelocity(distance);

        if (! customSpeed) speed = velocity;

        Pose pose = driveControl.getPose();
        double heading =  AngleUnit.normalizeRadians(pose.getHeading() - rotation);
        Pose newPose = new Pose(pose.getX(), pose.getY(), heading);

        if (! displayOnly) {
            if (! driveControl.isBusy()) {
                driveControl.moveToPose(newPose, 0.2, 3000);
            }
        }

        Logger.debug("current: %s   target: %s   pose: %s   new: %s   angle: %5.1f  rotation: %5.1f  ID: %d  distance: %4.1f  speed: %2.0f",
                current, target, pose, newPose, Math.toDegrees(angle), Math.toDegrees(rotation), id, distance, velocity);

        String msg = String.format("ID: %d  rotation: %5.1f  distance: %4.1f", id, Math.toDegrees(rotation), distance);
        displayAprilTagInfo(msg);
        displaySpeed();
        telemetry.update();
    }

    /**
     * Periodical update the telemetry information of position of the robot on the field based on the goal's april tag
     */
    private void updatePosition() {
        long time = System.currentTimeMillis();
        if (time - lastUpdate < 500) {
            return;
        }
        lastUpdate = time;

        lineUpWithGoal(true);
        telemetry.update();
    }

    /**
     * If the robot is moving a custom speed is of no longer set
     */
    private void updateCustomSpeed() {
        if (driveControl.isBusy())
            customSpeed = false;
    }

    /**
     * Set the color of the left and right LEDs
     * @param color the color of the LEDs
     */
    private void setLED(LEDColor color) {

        if (color == LEDColor.GREEN || color == LEDColor.NONE) {
            redLeftLED.off();
            redRightLED.off();
        }
        if (color == LEDColor.RED || color == LEDColor.NONE) {
            greenLeftLED.off();
            greenRightLED.off();
        }
        if (color == LEDColor.GREEN || color == LEDColor.YELLOW ) {
            greenLeftLED.on();
            greenRightLED.on();
        }
        if (color == LEDColor.RED || color == LEDColor.YELLOW) {
            redLeftLED.on();
            redRightLED.on();
        }
    }

    private void setLEDs (LEDColor leftColor, LEDColor rightColor) {
        setLED(redLeftLED, greenLeftLED, leftColor);
        setLED(redRightLED, greenRightLED, rightColor);
    }

    private void setLEDs(LEDColor color) {
        setLED(redLeftLED, greenLeftLED, color);
        setLED(redRightLED, greenRightLED, color);
    }

    private void setLED (LED redLED , LED greenLED, LEDColor color) {
        if (color == LEDColor.GREEN || color == LEDColor.NONE) {
            redLED.off();
        }
        if (color == LEDColor.RED || color == LEDColor.NONE) {
            greenLED.off();
        }
        if (color == LEDColor.GREEN || color == LEDColor.YELLOW ) {
            greenLED.on();
        }
        if (color == LEDColor.RED || color == LEDColor.YELLOW) {
            redLED.on();
        }
    }


    private void displayAprilTagInfo(String msg) {
        aprilTagMsg.setValue("%s", msg);
    }

    private void displaySpeed() {
        speedMsg.setValue("%4.0f", speed);
    }

    /**
     * Get the velocity of the launcher motors based on the distance to the goal
     *
     * @param distance distance to the goal in inches
     * @return velocity of the launcher motors
     */
    private double getVelocity(double distance) {

        double velocity = DEFAULT_SPEED;
        double minVelocity = 25;
        double[] distances = { 58, 69, 74, 82, 85, 120 };

        if (distance == 0)
            return velocity;

        velocity = minVelocity;
        for (int i = 1; i < distances.length; i++) {
            if (distance > distances[i-1] && distance <= distances[i]) {
                velocity = minVelocity + i;
            }
        }
        return velocity;
    }

    /**
     * Set the launcher motors speed based on the distance to the goal or the default speed if no april tag is found
     * or a custom speed, if one was set by the gamepad.
     */
    private void setSpeed() {

        if (! customSpeed) {
            if (distance == 0) {
                speed = DEFAULT_SPEED;
                Logger.warning("april tag not found, set to default speed: %5.2f", speed);
            } else {
                speed = getVelocity(distance);
                Logger.message("speed: %5.2f", speed);
            }
            launcher.setSpeed(speed);
        }
    }

    /**
     * Set the launcher motors speed based on the custom speed was set by the gamepad
     */
    private void setCustomSpeed() {
        launcher.setSpeed(speed);
        customSpeed = true;
    }

    /**
     * Fire one artifact
     */
    private void fire() {
        hopper.leverUp();
        launcher.fireLauncher();
    }

    /**
     * Fire all artifacts
     */
    private void fireAll() {
        hopper.leverUp();
        launcher.fireAllArtifacts();
    }

    /**
     * Start or stop the launcher motors. If the launcher is being powered on
     * set the speed of the launcher motors and also power off the intake
     *
     * @param on true to power the launcher on, false to power the launcher off
     */
    private void powerLauncher(boolean on) {

        if (on) {
            powerIntake(false);
            launcher.setSpeed(speed);
            launcher.runLauncher();
        } else {
            launcher.stopLauncher();
        }
    }

    /**
     * Start or stop the intake motor. Ii the intake is being powered on, stop the launcher
     * motors,
     *
     * @param on true to power the intake on, false to power the intake off
     */
    private void powerIntake(boolean on) {

        if (on) {
            //powerLauncher(false);
            launcher.gateClose();
            hopper.leverDown();
            intake.on();
        } else {
            intake.off();
            hopper.leverUp();
            launcher.gateOpen();
        }
    }

    /**
     * Line up with the goal and fire all artifacts.
     *
     * @noinspection unused
     */
    private void fastLaunch() {

        long timeout = 3000;
        long start = System.currentTimeMillis();
        launcher.setSpeed(20);
        lineUpWithGoal(false);
        while (driveControl.isBusy()) {
            if (System.currentTimeMillis() - start > timeout)
                return;
        }
        setSpeed();
        launcher.fireAllArtifacts();
    }
}
