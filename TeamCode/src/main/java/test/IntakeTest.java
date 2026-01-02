package test;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import common.Drive;
import common.DriveControl;
import common.DriveGamepad;
import common.Floodgate;
import common.Hopper;
import common.Intake;
import common.Launcher;
import common.Logger;
import utils.Increment;

@TeleOp(name="Intake Test", group="Test")
@SuppressLint("DefaultLocale")

public class IntakeTest extends LinearOpMode {

    private Intake intake;
    private Hopper hopper;
    private Launcher launcher;

    Increment speedIncrement;
    double speed = 0.9;
    Floodgate floodgate;
    long time;

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

        intake = new Intake(this);
        hopper = new Hopper(this);
        launcher = new Launcher(this);

        floodgate = new Floodgate(this);

        Drive drive = new Drive(this);

        DriveControl driveControl = new DriveControl(this, drive);
        driveControl.reset();
        driveControl.start();

        DriveGamepad driveGamepad = new DriveGamepad(this, driveControl);
        driveGamepad.start();

        speedIncrement = new Increment(0.01, 0.02, 0.05);

        speedMsg = telemetry.addData("Motor speed", 0);
        setDisplaySpeed();

        telemetry.addData("\nControls", "\n" +
                "  a - start / stop intake motors\n" +
                "  left bumper - decrease motor speed\n" +
                "  right bumper - increase motor speed\n" +
                "\n");
        telemetry.update();
    }

    private void handleGamepad() {
        Gamepad gamepad = gamepad1;

        if (System.currentTimeMillis() - this.time > 1000) {
            time = System.currentTimeMillis();
            //Logger.message("current %f", floodgate.getCurrent());
            //floodgate.getCurrent();
        }

        if (gamepad.aWasPressed()) {
            intake.intakeToggle();

        } else if (gamepad.bWasPressed()) {
            hopper.leverToggle();

        } else if (gamepad.yWasPressed()) {
            launcher.gateToggle();

        } else if (gamepad.left_bumper) {
            // increase motor speed
            speedIncrement.reset();
            while (gamepad.left_bumper) {
                speed = Math.max(speed - speedIncrement.get(), 0);
                setDisplaySpeed();
                telemetry.update();
                intake.setSpeed(speed);
            }

        } else if (gamepad.right_bumper) {
            // decrease the motor speed
            speedIncrement.reset();
            while (gamepad.right_bumper) {
                speed = Math.min(speed + speedIncrement.get(), 120);
                setDisplaySpeed();
                telemetry.update();
            }
            intake.setSpeed(speed);
        }
    }

    private void setDisplaySpeed() {
        speedMsg.setValue("%5.2f percent", speed * 100);
    }
}
