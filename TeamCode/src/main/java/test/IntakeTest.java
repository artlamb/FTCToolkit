package test;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import common.Config;
import common.Logger;
import utils.Increment;

@TeleOp(name="Intake Test", group="Test")
@SuppressLint("DefaultLocale")

public class IntakeTest extends LinearOpMode {

    private DcMotorEx intake;
    Increment speedIncrement;
    double speed = 0.9;

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

        intake = hardwareMap.get(DcMotorEx.class, Config.INTAKE);
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

        if (gamepad.aWasPressed()) {
            if (intake.getPower() > 0)  {
                intake.setPower(0);
            } else {
                intake.setPower(speed);
            }

        } else if (gamepad.left_bumper) {
            // increase motor speed
            speedIncrement.reset();
            while (gamepad.left_bumper) {
                speed = Math.max(speed - speedIncrement.get(), 0);
                setDisplaySpeed();
                telemetry.update();
                intake.setPower(speed);
            }

        } else if (gamepad.right_bumper) {
            // decrease the motor speed
            speedIncrement.reset();
            while (gamepad.right_bumper) {
                speed = Math.min(speed + speedIncrement.get(), 120);
                setDisplaySpeed();
                telemetry.update();
            }
            intake.setPower(speed);
        }
    }

    private void setDisplaySpeed() {
        speedMsg.setValue("%5.2f percent", speed * 100);
    }
}
