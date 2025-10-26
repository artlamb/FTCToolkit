package test;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import common.ColorSensor;
import common.Launcher;
import common.Logger;
import utils.Increment;

@TeleOp(name="Launcher Test", group="Test")
@SuppressLint("DefaultLocale")

public class LauncherTest extends LinearOpMode {

    private Launcher launcher;
    private ColorSensor colorSensor;

    Increment speedIncrement;
    double speed = 0.2;

    Telemetry.Item speedMsg;

    @Override
    public void runOpMode() {
        try {
            initialize();

            waitForStart();

            while (opModeIsActive()) {
                colorSensor.update();
                handleGamepad();
            }
            colorSensor.enable(false);

        } catch (Exception e) {
            Logger.error(e, "Error");
        }
    }

    private void initialize() {
        launcher = new Launcher(this);
        launcher.start();

        colorSensor = new ColorSensor(this);

        speedIncrement = new Increment(0.01, 0.02, 0.05);

        speedMsg = telemetry.addData("Motor speed", 0);
        setDisplaySpeed(speedMsg);

        telemetry.addData("\nControls", "\n" +
                "  a - start / stop launcher motors\n" +
                "  x - open / close close loader gate\n" +
                "  y - pull trigger\n" +
                "  right trigger - fire artifact\n" +
                "  left trigger - fire all artifacts\n" +
                "  left bumper - decrease motor speed\n" +
                "  right bumper - increase motor speed\n" +
                "\n");
        telemetry.update();
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
