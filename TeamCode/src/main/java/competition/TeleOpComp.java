package competition;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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

    private Drive drive;
    private DriveGamepad driveGamepad;
    private DriveControl driveControl;
    private Launcher launcher;
    Limelight limelight;

    Increment speedIncrement;
    double speed = 0.20;

    Telemetry.Item speedMsg;

    @Override
    public void runOpMode() {
        try {
            initialize();

            telemetry.addLine("Press start");
            telemetry.update();

            waitForStart();

            while (opModeIsActive()) {
                handleGamepad();
            }

        } catch (Exception e) {
            Logger.error(e, "Error");
        }
    }

    private void initialize() {
        drive = new Drive(this);

        driveControl = new DriveControl(this, drive);
        driveControl.start();

        driveGamepad = new DriveGamepad(this, driveControl);
        driveGamepad.start();

        launcher = new Launcher(this);
        launcher.start();

        limelight = new Limelight(this);
        limelight.setPipeline(Limelight.Pipeline.APRIL_TAG);

        speedIncrement = new Increment(0.01, 0.02, 0.05);

        speedMsg = telemetry.addData("Motor speed", 0);

        telemetry.addData("\nControls", "\n" +
                "  a - start / stop launcher motors\n" +
                "  x - open / close close loader gate\n" +
                "  y - pull trigger\n" +
                "  right trigger - fire artifact\n" +
                "  left trigger - fire all artifacts\n" +
                "  left bumper - decrease motor speed\n" +
                "  right bumper - increase motor speed\n" +
                "\n");
    }

    private void handleGamepad() {
        Gamepad gamepad = gamepad1;

        if (gamepad.aWasPressed()) {
            if (launcher.isRunning()) {
                launcher.stopLauncher();
            } else {
                launcher.setSpeed(speed);
                launcher.runLauncher();
            }

        } else if (gamepad.xWasPressed()) {
            if (launcher.loaderIsOpen()) {
                launcher.closeLoader();
            } else {
                launcher.openLoader();
            }

        } else if (gamepad.yWasPressed()) {
            launcher.pullTrigger();

        } else if (gamepad.bWasPressed()) {
            double angle = limelight.GetTx();
            Pose pose = driveControl.getPose();
            Logger.message("%s", pose.toString());

            double heading = AngleUnit.normalizeRadians(pose.getHeading() - Math.toRadians(angle));
            pose = new Pose(pose.getX(), pose.getY(), heading);
            driveControl.moveToPose(pose,0.2, 1000);
            Logger.message("angle: %5.2f  pose: %s", angle, pose.toString());

        } else if (gamepad.right_trigger > 0) {
            launcher.fireLauncher();
            while (gamepad.right_trigger > 0) {
                Thread.yield();
            }

        } else if (gamepad.left_trigger > 0) {
            launcher.fireAllArtifacts();
            while (gamepad.left_trigger > 0) {
                Thread.yield();
            }

        } else if (gamepad.left_bumper) {
            // increase motor speed
            speedIncrement.reset();
            while (gamepad.left_bumper) {
                speed = Math.max(speed - speedIncrement.get(), 0);
                setDisplaySpeed(speedMsg);
                telemetry.update();
            launcher.setSpeed(speed);
            }

        } else if (gamepad.right_bumper) {
            // decrease the motor speed
            speedIncrement.reset();
            while (gamepad.right_bumper) {
                speed = Math.min(speed + speedIncrement.get(), 0.95);
                setDisplaySpeed(speedMsg);
                telemetry.update();
            }
            launcher.setSpeed(speed);
        }
    }

    private void setDisplaySpeed(Telemetry.Item item) {
        item.setValue("%5.3f", speed);
    }
}
