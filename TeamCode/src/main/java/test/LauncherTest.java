package test;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Arrays;
import java.util.List;

import common.Config;
import common.Floodgate;
import common.Launcher;
import common.Logger;
import utils.Increment;

@TeleOp(name="Launcher Test", group="Test")
@SuppressLint("DefaultLocale")
@Disabled
@com.acmerobotics.dashboard.config.Config

public class LauncherTest extends LinearOpMode {

    //public static double pidP = 40;
    //public static double pidI = 0;
    //public static double pidF = 14.5;
    public static double runSpeed = 27;
    public static double idleSpeed = 25;
    public static PIDFCoefficients coefficientsLeft  = new PIDFCoefficients(50, 0, 0, 14.5);
    public static PIDFCoefficients coefficientsRight = new PIDFCoefficients(50, 0, 0, 14.5);

    private Launcher launcher;
    Floodgate floodgate;

    Increment speedIncrement;
    private double speed = runSpeed;
    boolean running = false;
    boolean idling = false;
    boolean speedChanged = false;

    private double lastLeftError = 0;
    private double lastRightError = 0;
    private long   leftTime = 0;
    private long   rightTime = 0;
    private long   spinUpStartTime;

    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    public List<DcMotorEx> motors;

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
        leftMotor = hardwareMap.get(DcMotorEx.class, Config.LAUNCHER_LEFT);
        rightMotor = hardwareMap.get(DcMotorEx.class, Config.LAUNCHER_RIGHT);
        motors = Arrays.asList(leftMotor, rightMotor);


        launcher = new Launcher(this);
        launcher.start();

        floodgate = new Floodgate(this);

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

        if (gamepad.aWasPressed()) {
            runLauncher(! running, runSpeed);

        } else if (gamepad.bWasPressed()) {
            runLauncher(true, (idling ? runSpeed : idleSpeed));
            idling = ! idling;

        } else if (gamepad.xWasPressed()) {
            running = ! running;

        } else if (gamepad.yWasPressed()) {
            setPID();

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
                speed = Math.min(speed + speedIncrement.get(), 120);
                setDisplaySpeed(speedMsg);
                telemetry.update();
            }
            launcher.setSpeed(speed);
        }
    }

    private void updateSpeed(double speed) {
        double VELOCITY_MULTIPLIER = 20;
        double velocity = VELOCITY_MULTIPLIER * speed;
        leftMotor.setVelocity(velocity);
        rightMotor.setVelocity(velocity);
        running = speed != 0;
        spinUpStartTime = System.currentTimeMillis();
        speedChanged = true;
        this.speed = speed;
    }

    private void update() {
        if (running) {
            double VELOCITY_MULTIPLIER = 20;
            double velocity = VELOCITY_MULTIPLIER * speed;
            double leftVelocity = leftMotor.getVelocity();
            double rightVelocity = rightMotor.getVelocity();

            double leftError = (velocity - leftVelocity) / VELOCITY_MULTIPLIER;
            double rightError = (velocity - rightVelocity) / VELOCITY_MULTIPLIER;

            long time = System.currentTimeMillis();

            if (speedChanged) {
                if (Math.abs(leftError) <= 1 && Math.abs(rightError) <= 1 &&  Math.abs(leftError + rightError) <= 2) {
                    Logger.info("launcher spin up complete after %d ms",
                            time - spinUpStartTime);
                    speedChanged = false;
                }
            }

            if (leftError != lastLeftError || rightError != lastRightError) {
                String stableLeft = "";
                String stableRight = "";
                if (leftError != lastLeftError) {
                    stableLeft = String.format("%6d ms", time - leftTime);
                    leftTime = time;
                }
                if (rightError != lastRightError) {
                    stableRight += String.format("%6d ms", time - rightTime);
                    rightTime = time;
                }


                double leftCurrent = leftMotor.getCurrent(CurrentUnit.AMPS);
                double rightCurrent = rightMotor.getCurrent(CurrentUnit.AMPS);

                Logger.verbose("spin up: %6d ms  stable: %10s  %10s  delta velocity  left: %5.0f  right: %5.0f    current  left: %5.2f  right: %5.2f",
                        time - spinUpStartTime,
                        stableLeft, stableRight,
                        leftError, rightError,
                        leftCurrent,
                        rightCurrent
                        //(leftCurrent + rightCurrent) /floodgate.getCurrent()
                );

                lastLeftError = leftError;
                lastRightError = rightError;
            }
            //floodgate.display(100);

        }
    }

    private void setPID( ) {
        //PIDFCoefficients coefficients = new PIDFCoefficients(pidP, pidI, 0, pidF);
        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficientsLeft);
        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficientsRight);
        updateSpeed(runSpeed);
        }

       private void runLauncher(boolean run, double speed)  {
            if (run) spinUpStartTime = System.currentTimeMillis();
            updateSpeed(run ? speed : 0);
            Logger.debug("launcher running: %b  speed: %5.0f", run, speed);
       }

    private void setDisplaySpeed(Telemetry.Item item) {
        item.setValue("%5.3f", speed);
    }

}
