package competition;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

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

public class TeleOpComp extends LinearOpMode {

    private DriveControl driveControl;
    private Launcher launcher;
    Limelight limelight;

    private double DEFAULT_SPEED = 28;
    Increment speedIncrement;
    double speed = DEFAULT_SPEED;

    Telemetry.Item speedMsg;
    Telemetry.Item areaMsg;

    long lastUpdate;
    double lastArea;

    @Override
    public void runOpMode() {
        try {
            initialize();

            telemetry.addLine("Press start");
            telemetry.update();

            waitForStart();

            while (opModeIsActive()) {
                handleGamepad();
                updateTargetArea();
            }

        } catch (Exception e) {
            Logger.error(e, "Error");
        }
    }

    private void initialize() {
        Drive drive = new Drive(this);

        driveControl = new DriveControl(this, drive);
        driveControl.start();

        DriveGamepad driveGamepad = new DriveGamepad(this, driveControl);
        driveGamepad.start();

        launcher = new Launcher(this);
        launcher.start();
        launcher.setSpeed(speed);

        limelight = new Limelight(this);
        limelight.setPipeline(Limelight.Pipeline.APRIL_TAG);

        speedIncrement = new Increment(1, 2, 3);

        speedMsg = telemetry.addData("Motor speed", 0);
        displaySpeed();

        areaMsg = telemetry.addData("Target Area", 0);

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
            lineUpWithAprilTag();

        } else if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()) {
            setSpeed();
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

    private void lineUpWithAprilTag() {
        double angle = limelight.GetTx();
        Pose pose = driveControl.getPose();
        double heading = AngleUnit.normalizeRadians(pose.getHeading() - Math.toRadians(angle));
        Pose newPose = new Pose(pose.getX(), pose.getY(), heading);
        driveControl.moveToPose(newPose,0.2, 1000);
        Logger.message("angle: %5.2f  current: %s   new: %s", angle, pose.toString(), newPose.toString());
    }

    private void lineUpWithGoal() {
        Pose current = driveControl.getPose();

        // aim for the center of the goal
        Pose center;
        if (current.getHeading() > Math.PI / 2) {
            center = new Pose(-60, 60, 135);
        } else {
            center = new Pose(60, 60, 45);
        }
        double a = center.getX() - current.getX();
        double b = center.getY() - current.getY();
        double angle = Math.atan2(b, a);
        Pose newPose = new Pose(current.getX(), current.getY(), angle);
        //driveControl.moveToPose(newPose, 0.2, 1000);
        Logger.debug("current: %s  corner: %s  pose: %s", current, center, newPose);
    }

    private void updatePosition() {

        Pose pose = driveControl.getPose();
        Pose newPose = limelight.getPosition(pose.getHeading());
        if (newPose != null) {
            Logger.message("new pose: %s", newPose);
            //driveControl.setPose(newPose);
        }
    }

    private void displaySpeed() {
        speedMsg.setValue("%4.0f", speed);
    }

    private void setSpeed() {
        double area = limelight.GetTargetArea();
        if (area <= 0) {
            speed = DEFAULT_SPEED;
            Logger.warning("april tag not found, set to defaukt speed: %5.2f", speed);
            return;
        }

        speed = Math.round(25.05 * Math.pow(area-0.4, -0.09) + 2);
        Logger.message("speed: %5.2f", speed);
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

        areaMsg.setValue("%5.2f", area);
        Logger.message("Limelight target Area: %5.2f", area);
        telemetry.update();
    }
}
