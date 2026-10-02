package test;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import common.Config;
import common.Floodgate;
import common.Launcher;
import common.Logger;
import utils.Increment;

@TeleOp(name="Launcher Test", group="Test")
@SuppressLint("DefaultLocale")
//@Disabled
@com.acmerobotics.dashboard.config.Config

public class LauncherOneMotor extends LinearOpMode {

    public static double runSpeed = 27;
    public static double idleSpeed = 25;
    public static PIDFCoefficients coefficientsLeft  = new PIDFCoefficients(50, 0, 0, 14.5);

    //private Launcher launcher;
    Floodgate floodgate;

    Increment speedIncrement;
    private double speed = runSpeed;
    boolean running = false;
    boolean idling = false;
    boolean speedChanged = false;

    private double lastError = 0;
    private long   leftTime = 0;
    private long   spinUpStartTime;

    private DcMotorEx motor;

    Telemetry.Item speedMsg;

    @Override
    public void runOpMode() {
        try {
            initialize();

            waitForStart();

            while (opModeIsActive()) {
                handleGamepad();
                update();
            }

        } catch (Exception e) {
            Logger.error(e, "Error");
        }
    }

    private void initialize() {
        motor = hardwareMap.get(DcMotorEx.class, Config.LAUNCHER_LEFT);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //launcher = new Launcher(this);
        //launcher.start();

        //floodgate = new Floodgate(this);

        speedIncrement = new Increment(1, 2, 3);

        speedMsg = telemetry.addData("Motor speed", 0);
        setDisplaySpeed(speedMsg);

        telemetry.addData("\nControls", "\n" +
                "  a - start / stop launcher motors\n" +
                "  b - run / idle launcher motors\n" +
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

        // Start the launcher motor
        if (gamepad.aWasPressed()) {
            runLauncher(! running, runSpeed);

        // Stop the launcher motor
        } else if (gamepad.bWasPressed()) {
            runLauncher(true, (idling ? runSpeed : idleSpeed));
            idling = ! idling;

        // Open / close the launcher gate
        } else if (gamepad.xWasPressed()) {
            running = ! running;

        // Change the PID coefficients of the launcher motors
        } else if (gamepad.yWasPressed()) {
            setPID();

        // Pull the trigger
        } else if (gamepad.right_trigger > 0) {
            //launcher.fireLauncher();
            while (gamepad.right_trigger > 0) {
                Thread.yield();
            }

        // Fire all the artifacts
        } else if (gamepad.left_trigger > 0) {
            //launcher.fireAllArtifacts();
            while (gamepad.left_trigger > 0) {
                Thread.yield();
            }

        // increase motor speed
        } else if (gamepad.left_bumper) {
            speedIncrement.reset();
            while (gamepad.left_bumper) {
                speed = Math.max(speed - speedIncrement.get(), 0);
                setDisplaySpeed(speedMsg);
                telemetry.update();
            updateSpeed(speed);
            }

        // decrease the motor speed
        } else if (gamepad.right_bumper) {
            speedIncrement.reset();
            while (gamepad.right_bumper) {
                speed = Math.min(speed + speedIncrement.get(), 120);
                setDisplaySpeed(speedMsg);
                telemetry.update();
            }
            updateSpeed(speed);
        }
    }

    private void updateSpeed(double speed) {

        double VELOCITY_MULTIPLIER = 20;
        double velocity = VELOCITY_MULTIPLIER * speed;
        motor.setVelocity(velocity);

        Logger.message("launcher speed set to %f  velocity set to %f", speed, velocity);
        //launcher.setSpeed(speed);

        running = speed != 0;
        spinUpStartTime = System.currentTimeMillis();
        speedChanged = true;
        this.speed = speed;
    }

    private void update() {
        if (running) {
            double VELOCITY_MULTIPLIER = 20;
            double velocity = VELOCITY_MULTIPLIER * speed;
            double motorVelocity = motor.getVelocity();

            double leftError = (velocity - motorVelocity) / VELOCITY_MULTIPLIER;

            long time = System.currentTimeMillis();

            if (speedChanged) {
                if (Math.abs(leftError) <= 1) {
                    Logger.info("launcher spin up complete after %d ms", time - spinUpStartTime);
                    speedChanged = false;
                }
            }

            if (leftError != lastError) {
                String stable = "";
                if (leftError != lastError) {
                    stable = String.format("%6d ms", time - leftTime);
                    leftTime = time;
                }

                double leftCurrent = motor.getCurrent(CurrentUnit.AMPS);

                Logger.verbose("spin up: %6d ms  stable: %10s  delta velocity : %5.0f  current:  %5.2f  ",
                        time - spinUpStartTime,
                        stable,
                        leftError,
                        leftCurrent
                        //floodgate.getCurrent()
                );

                lastError = leftError;
            }
            //floodgate.display(100);
        }
    }

    private void setDisplaySpeed(Telemetry.Item item) {
        item.setValue("%5.3f", speed);
    }

    private void setPID( ) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficientsLeft);
        updateSpeed(runSpeed);
        }

    private void runLauncher(boolean run, double speed)  {
            if (run) spinUpStartTime = System.currentTimeMillis();
            updateSpeed(run ? speed : 0);
            Logger.debug("launcher running: %b  speed: %5.0f", run, speed);
       }
}
