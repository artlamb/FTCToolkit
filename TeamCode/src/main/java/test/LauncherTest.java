package test;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import common.Launcher;
import common.Logger;
import utils.Increment;

@TeleOp(name="Launcher Test", group="Test")
@SuppressLint("DefaultLocale")

public class LauncherTest extends LinearOpMode {

    private Launcher launcher;
    Increment speedIncrement;
    double speed = 0.2;

    Telemetry.Item speedMsg;

    @Override
    public void runOpMode() {
        try {
            initialize();

            waitForStart();

            while (opModeIsActive()) {
                handleGamepad();
            }

        } catch (Exception e) {
            Logger.error(e, "Error");
        }
    }

    private void initialize() {
        launcher = new Launcher(this);
        launcher.start();

        speedIncrement = new Increment(0.01, 0.02, 0.05);

        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        speedMsg = telemetry.addData("Motor speed", 0);
        setDisplaySpeed(speedMsg);

        telemetry.addData("\nControls", "\n" +
                "  a - start launcher motors\n" +
                "  x - stop launcher motors\n" +
                "  y - fire artifact\n" +
                "  left bumper - decrease motor speed\n" +
                "  right bumper - increase motor speed\n" +
                "\n");
        telemetry.update();
    }

    private void handleGamepad() {
        if (gamepad1.aWasPressed()) {
            launcher.setSpeed(speed);
            launcher.runLauncher();

        } else if (gamepad1.xWasPressed()) {
            launcher.stopLauncher();

        } else if (gamepad1.yWasPressed()) {
            launcher.fireLauncher();

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
