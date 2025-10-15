package competition;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import common.Drive;
import common.DriveControl;
import common.DriveGamepad;
import common.Launcher;
import common.Limelight;
import common.Logger;
import utils.Increment;
import utils.Pose;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Competition")
@SuppressLint("DefaultLocale")

public class TeleOp extends LinearOpMode {


    private Drive drive;
    private DriveGamepad driveGamepad;
    private DriveControl driveControl;
    private Launcher launcher;
    Limelight limelight;

    Increment speedIncrement;
    double speed = 0.1;

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

        speedIncrement = new Increment(0.01, 0.02, 0.05);

        speedMsg = telemetry.addData("Motor speed", 0);

        telemetry.addData("\nControls", "\n" +
                "  a - start launcher motors\n" +
                "  x - stop launcher motors\n" +
                "  y - fire artifact\n" +
                "  left bumper - decrease motor speed\n" +
                "  right bumper - increase motor speed\n" +
                "\n");
    }

    private void handleGamepad() {
        if (gamepad1.aWasPressed()) {
            launcher.setSpeed(speed);
            launcher.runLauncher();

        } else if (gamepad1.xWasPressed()) {
            launcher.stopLauncher();

        } else if (gamepad1.yWasPressed()) {
            launcher.fireLauncher();

        } else if (gamepad1.bWasPressed()) {
            double angle = limelight.GetTx();
            Pose pose = driveControl.getPose();
            double heading = pose.getHeading() + Math.toRadians(angle);
            pose = new Pose(pose.getX(), pose.getY(), heading);
            driveControl.moveToPose(pose,0.2, 1000);
            Logger.message("angle: %5.2f  heading: %5.2f", angle, Math.toDegrees(heading));

        } else if (gamepad1.left_bumper) {
            // increase motor speed
            speedIncrement.reset();
            while (gamepad1.left_bumper) {
                speed = Math.max(speed - speedIncrement.get(), 0);
                setDisplaySpeed(speedMsg);
                telemetry.update();
            launcher.setSpeed(speed);
            }

        } else if (gamepad1.right_bumper) {
            // decrease the motor speed
            speedIncrement.reset();
            while (gamepad1.right_bumper) {
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
