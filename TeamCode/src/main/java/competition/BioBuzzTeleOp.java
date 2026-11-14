package competition;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import common.Config;
import common.Drive;
import common.DriveControl;
import common.DriveGamepad;
import common.Floodgate;
import common.Intake;
import common.Logger;
import common.Trigger;
import utils.Increment;

@TeleOp(name="BioBuzz TeleOp", group="Competition")
@SuppressLint("DefaultLocale")
//@Disabled
@com.acmerobotics.dashboard.config.Config

public class BioBuzzTeleOp extends LinearOpMode {

    public static double runSpeed = 75;
    public static double idleSpeed = 75;
    //public static PIDFCoefficients coefficientsLeft  = new PIDFCoefficients(50, 0, 0, 14.5);
    public static PIDFCoefficients pidfLauncher  = new PIDFCoefficients(40, 1, 0, 0);

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

    double intakeSpeed = 0.3;

    private DcMotorEx motor;
    private Intake intake;
    private CRServo triggerServo;
    private boolean servoOn = false;

    private Trigger trigger;

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
        motor = hardwareMap.get(DcMotorEx.class, Config.LAUNCHER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = new Intake(this);
        intake.setSpeed(intakeSpeed);

        //triggerServo = hardwareMap.get(CRServo.class, Config.TRIGGER);

        Drive drive = new Drive(this);

        DriveControl driveControl = new DriveControl(this, drive);
        driveControl.reset();
        driveControl.start();

        DriveGamepad driveGamepad = new DriveGamepad(this, driveControl);
        driveGamepad.start();

        trigger = new Trigger(this);
        trigger.start();

        speedIncrement = new Increment(1, 2, 3);

        speedMsg = telemetry.addData("Motor speed", 0);
        setDisplaySpeed(speedMsg);

        telemetry.addData("\nControls", "\n" +
                "  a - start / stop launcher motors\n" +
                "  x - reverse everything\n" +
                "  y - toggle intake\n" +
                "  right trigger - fire pollen\n" +
                "  left bumper - decrease motor speed\n" +
                "  right bumper - increase motor speed\n" +
                "\n");
        telemetry.update();
    }

    private void handleGamepad() throws InterruptedException {
        Gamepad gamepad = gamepad2;

        // Start the launcher motor
        if (gamepad.aWasPressed()) {
            runLauncher(! running, runSpeed);

            // Stop the launcher motor
        } else if (gamepad.bWasPressed()) {
            //runLauncher(true, (idling ? runSpeed : idleSpeed));
            //idling = ! idling;

            // Reverse everything for 2 seconds
        } else if (gamepad.xWasPressed()) {
            intake.off();
            trigger.off();
            runLauncher(false, 0);

            intake.reverse();
            trigger.reverse();
            runLauncher(true, -20);
            Thread.sleep(2000);

            runLauncher(false, 0);
            trigger.off();
            intake.off();

            //running = ! running;

            // Change the PID coefficients of the launcher motors
            //} else if (gamepad.yWasPressed()) {
            //    setPID();

        } else if (gamepad.yWasPressed()) {
            intake.toggle();

            // Pull the trigger
        } else if (gamepad.right_trigger > 0) {
            servoOn = !servoOn;
            //triggerServo.setPower(servoOn ? 1.0 : 0.0);
            trigger.on();
            while (gamepad.right_trigger > 0) {
                Thread.yield();
            }

            // Fire all the artifacts
        } else if (gamepad.left_trigger > 0) {
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
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfLauncher);
        updateSpeed(runSpeed);
    }

    private void runLauncher(boolean run, double speed)  {
        if (run) spinUpStartTime = System.currentTimeMillis();
        updateSpeed(run ? speed : 0);
        Logger.debug("launcher running: %b  speed: %5.0f", run, speed);
    }
}
