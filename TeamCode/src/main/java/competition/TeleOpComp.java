package competition;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import common.Config;
import common.Drive;
import common.DriveControl;
import common.DriveGamepad;
import common.Launcher;
import common.Limelight;
import common.Logger;
import utils.Increment;
import utils.Pose;

@TeleOp(name="TeleOpComp", group="Competition")
@SuppressLint("DefaultLocale")
@com.acmerobotics.dashboard.config.Config

public class TeleOpComp extends LinearOpMode {

    public static double coefficientV = 1;

    private DriveControl driveControl;
    private Launcher launcher;
    private Limelight limelight;
    private LED redLeftLED;
    private LED redRightLED;
    private LED greenLeftLED;
    private LED greenRightLED;

    private final double DEFAULT_SPEED = 28;
    Increment speedIncrement;
    double speed = DEFAULT_SPEED;

    Telemetry.Item speedMsg;
    Telemetry.Item aprilTagMsg;

    long lastUpdate;
    double lastArea;
    double distance = 0;

    int aprilTagID = 0;

    enum LEDState { GREEN, RED, NONE }
    @Override
    public void runOpMode() {
        try {
            initialize();

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            telemetry.update();

            setLED(LEDState.NONE);

            waitForStart();

            while (opModeIsActive()) {
                handleGamepad();
                updatePosition();
            }

        } catch (Exception e) {
            Logger.error(e, "Error");
        }
    }

    private void initialize() {
        Drive drive = new Drive(this);

        driveControl = new DriveControl(this, drive);
        driveControl.reset();
        driveControl.start();

        DriveGamepad driveGamepad = new DriveGamepad(this, driveControl);
        driveGamepad.start();

        launcher = new Launcher(this);
        launcher.start();
        launcher.setSpeed(speed);

        limelight = new Limelight(this);
        limelight.setPipeline(Limelight.Pipeline.LOCATION);

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
                "  y - pull trigger\n" +
                "  dpad up - auto set motor speed\n" +
                "  right trigger - fire artifact\n" +
                "  left trigger - fire all artifacts\n" +
                "  left bumper - decrease motor speed\n" +
                "  right bumper - increase motor speed\n" +
                "\n");
    }

    private void handleGamepad() {

        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            if (launcher.isRunning()) {
                launcher.stopLauncher();
            } else {
                launcher.setSpeed(speed);
                launcher.runLauncher();
            }

        } else if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) {
            if (launcher.loaderIsOpen()) {
                launcher.closeLoader();
            } else {
                launcher.openLoader();
            }

        } else if (gamepad1.yWasPressed() || gamepad2.yWasPressed()) {
            launcher.pullTrigger();

        } else if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
            lineUpWithGoal(false);
            setSpeed();

        } else if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()) {
            setSpeedFromTargetArea();
            displaySpeed();

        } else if (gamepad1.right_trigger > 0) {
            launcher.fireLauncher();
            while (gamepad1.right_trigger > 0) {
                Thread.yield();
            }
        } else if (gamepad2.right_trigger > 0) {
            launcher.fireLauncher();
            while (gamepad2.right_trigger > 0) {
                Thread.yield();
            }

        } else if (gamepad1.left_trigger > 0) {
            launcher.fireAllArtifacts();
            while (gamepad1.left_trigger > 0) {
                Thread.yield();
            }
        } else if (gamepad2.left_trigger > 0) {
            launcher.fireAllArtifacts();
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
                launcher.setSpeed(speed);
            }
        } else if (gamepad2.left_bumper) {
            // increase motor speed
            speedIncrement.reset();
            while (gamepad2.left_bumper) {
                speed = Math.max(speed - speedIncrement.get(), 0);
                displaySpeed();
                telemetry.update();
                launcher.setSpeed(speed);
            }

        } else if (gamepad1.right_bumper) {
            // decrease the motor speed
            speedIncrement.reset();
            while (gamepad1.right_bumper) {
                speed = Math.min(speed + speedIncrement.get(), 140);
                displaySpeed();
                telemetry.update();
            }
            launcher.setSpeed(speed);
        } else if (gamepad2.right_bumper) {
            // decrease the motor speed
            speedIncrement.reset();
            while (gamepad2.right_bumper) {
                speed = Math.min(speed + speedIncrement.get(), 140);
                displaySpeed();
                telemetry.update();
            }
            launcher.setSpeed(speed);
        }
    }

    private void lineUpWithGoal(boolean displayOnly) {

        distance = 0;

        // aim for the center of the goal
        int id = limelight.GetAprilTagID();
        Pose target;
        double num = 70.5 - 6;
        if (id == 20) {  // blue
            target = new Pose(-num, num, Math.toRadians(135));
        } else if (id == 24) {     //red
            target = new Pose(num, num, Math.toRadians(45));
        } else {
            setLED(LEDState.RED);
            displayAprilTagInfo("april tag if not found");

            if (! displayOnly)
                Logger.message("april tag if not found");
            return;
        }

        Pose current = limelight.getPosition();
        if (current == null) {
            setLED(LEDState.RED);
            displayAprilTagInfo("april tag if not found");
            if (! displayOnly)
                Logger.message("no april tag found");
            return;
        }

        setLED(LEDState.GREEN);

        double a = target.getX() - current.getX();
        double b = target.getY() - current.getY();
        double angle = Math.atan2(b, a);
        double rotation = AngleUnit.normalizeRadians(current.getHeading() - angle);

        distance = Math.abs(Math.hypot(a, b));
        speed = getVelocity(distance);

        Pose pose = driveControl.getPose();
        double heading =  AngleUnit.normalizeRadians(pose.getHeading() - rotation);
        Pose newPose = new Pose(pose.getX(), pose.getY(), heading);

        if (! displayOnly) {
            driveControl.moveToPose(newPose, 0.2, 3000);
        }

        Logger.debug("current: %s   target: %s   pose: %s   new: %s   angle: %5.1f  rotation: %5.1f  ID: %d  distance: %4.1f  speed: %2.0f",
                current, target, pose, newPose, Math.toDegrees(angle), Math.toDegrees(rotation), id, distance, speed);

        String msg = String.format("ID: %d  rotation: %5.1f  distance: %4.1f", id, Math.toDegrees(rotation), distance);
        displayAprilTagInfo(msg);
        telemetry.update();
    }

    private void updatePosition() {
        long time = System.currentTimeMillis();
        if (time - lastUpdate < 500) {
            return;
        }
        lastUpdate = time;

        lineUpWithGoal(true);
        telemetry.update();
    }

    private void setLED(LEDState state) {

        if (state == LEDState.GREEN || state == LEDState.NONE) {
            redLeftLED.off();
            redRightLED.off();
        }
        if (state == LEDState.RED || state == LEDState.NONE) {
            greenLeftLED.off();
            greenRightLED.off();
        }
        if (state == LEDState.GREEN) {
            greenLeftLED.on();
            greenRightLED.on();
        }
        if (state == LEDState.RED) {
            redLeftLED.on();
            redRightLED.on();
        }
    }

    private void displayAprilTagInfo(String msg) {
        aprilTagMsg.setValue("%s", msg);
    }

    private void displaySpeed() {
        speedMsg.setValue("%4.0f", speed);
    }

    private void setSpeed() {

        if (distance == 0) {
            speed = DEFAULT_SPEED;
            Logger.warning("april tag not found, set to default speed: %5.2f", speed);
            return;
        }

        //speed = Math.round(25.05 * Math.pow(area-0.4, -0.09) + 2);
        Logger.message("speed: %5.2f", speed);
    }

    private void setSpeedFromTargetArea() {

        double area = limelight.GetTargetArea();
        if (area <= 0) {
            speed = DEFAULT_SPEED;
            Logger.warning("april tag not found, set to default speed: %5.2f", speed);
            return;
        }

        speed = Math.round(25.05 * Math.pow(area-0.4, -0.09) + 2);
        Logger.message("speed: %5.2f", speed);
    }

    private void lineUpWithAprilTag() {
        double angle = limelight.GetTx();
        Pose pose = driveControl.getPose();
        double heading = AngleUnit.normalizeRadians(pose.getHeading() - Math.toRadians(angle));
        Pose newPose = new Pose(pose.getX(), pose.getY(), heading);
        driveControl.moveToPose(newPose,0.2, 1000);
        Logger.message("angle: %5.2f  current: %s   new: %s", angle, pose.toString(), newPose.toString());
    }

    private void updateTargetArea() {
        long time = System.currentTimeMillis();
        if (time - lastUpdate < 500) {
            return;
        }
        lastUpdate = time;

        double area = limelight.GetTargetArea();
        if (area == lastArea) {
            return;
        }
        lastArea = area;

        aprilTagMsg.setValue("%5.2f", area);
        Logger.message("Limelight target Area: %5.2f", area);
        telemetry.update();
    }

    private void trajectory(double distance) {

        double gravity = 9.80665;
        double y = DistanceUnit.INCH.toMeters(20) ;
        double r = DistanceUnit.INCH.toMeters(distance);
        double angle = Math.toRadians(60);

        double v = r / (Math.sqrt(2 * (r * Math.tan(angle) - y) / gravity) * Math.cos(angle));

        Logger.message("velocity: %5.2f  %5.2f", v, v * coefficientV);
    }

    private double getVelocity(double distance) {

        double velocity = DEFAULT_SPEED;
        double minVelocity = 25;
        double[] distances = { 55, 59, 71, 79, 85 };

        if (distance == 0)
            return velocity;

        for (int i = 0; i < distances.length; i++) {
            if (distances[i] <= distance) {
                velocity = minVelocity + i;
            }
        }
        return velocity;
    }
}
