package tools;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.SortedSet;

import common.Logger;
import utils.Increment;

@com.acmerobotics.dashboard.config.Config
@TeleOp(name="Calibrate Motor", group="Test")
//@SuppressWarnings({"FieldCanBeLocal", "unused"})

public class CalibrateMotor extends LinearOpMode {

    private static class MotorInfo implements Comparable<MotorInfo>{
        String      name;
        DcMotor     motor;
        int         home;
        int         target;
        double      speed;

        @Override
        public int compareTo(MotorInfo o) {
            // Method the sort the list of motors, sort by port number.
            //return name.compareTo(o.name);
            return motor.getPortNumber() - o.motor.getPortNumber();
        }
    }
    MotorInfo[] motors = new MotorInfo[8];
    int motorCount;
    int currentMotor;

    private static class MotorSettings {
        String              name;
        int                 home;
        int                 target;
        double              speed;
        DcMotor.Direction   direction;
    }

    private static class Settings {
        String currentMotor;
        ArrayList<MotorSettings> motors;
    }

    public static double  speed = 0.25;
    public static boolean holdPower = true;

    private static final String settingsDir = String.format("%s/%s/", AppUtil.FIRST_FOLDER.getAbsolutePath(), "settings");

    public enum MOTOR_SELECT { PREVIOUS, NEXT }

    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotor motor   = null;

    @Override
    public void runOpMode() {

        Increment speedIncrement = new Increment(0.01, 0.02, 0.05);
        Increment incrementFine = new Increment(1, 10, 30);
        Increment incrementCoarse = new Increment(10, 50, 100);

        getMotors();
        readSettings();

        telemetry.addLine("Press start");
        telemetry.update();
        telemetry.setAutoClear(false);

        waitForStart();

        Telemetry.Item motorNameMsg =  telemetry.addData("Motor name", 0);
        Telemetry.Item directionMsg = telemetry.addData("Motor direction", 0);
        Telemetry.Item brakingMsg = telemetry.addData("Motor braking", 0);
        Telemetry.Item speedMsg = telemetry.addData("Motor speed", 0);
        Telemetry.Item positionMsg = telemetry.addData("Motor position", 0);
        Telemetry.Item homeMsg = telemetry.addData("Home position", 0);
        Telemetry.Item targetMsg = telemetry.addData("Target position", 0);

        setDisplayName (motorNameMsg);
        setDisplayDirection(directionMsg);
        setDisplaySpeed(speedMsg);
        setDisplayBraking(brakingMsg);
        setDisplayPosition(positionMsg);
        setDisplayHome(homeMsg);
        setDisplayTarget(targetMsg);

        telemetry.addData("\nMotor Calibration Controls", "\n" +
                "  left stick - increase/decrease home position\n" +
                "  right stick - increase/decrease target position\n" +
                "  x - run to home position\n" +
                "  b - run servo to target position\n" +
                "  y - set home position to current position\n" +
                "  a - set target position to current position\n" +
                "  dpad left - select previous motor\n" +
                "  dpad right - select next motor\n" +
                "  dpad up - change motor direction\n" +
                "  dpad down - change motor breaking\n" +
                "  left trigger - run motor backwards\n" +
                "  right trigger - run the motor forward\n" +
                "  left bumper - decrease motor speed\n" +
                "  right bumper - increase motor speed\n" +
                "  left stick button - zero encoders" +
                "  back - exit without saving\n" +
                "\n");

        telemetry.update();
        boolean save = true;

        while (opModeIsActive()) {

            if (gamepad1.a) {
                // set target position to current position
                motors[currentMotor].target = motor.getCurrentPosition();

            } else if (gamepad1.y) {
                // set the home position to the current position
                motors[currentMotor].home = motor.getCurrentPosition();

            } else if (gamepad1.x) {
                // run to home position
                runToPosition(motors[currentMotor].home);
                while (gamepad1.x) sleep(10);

            } else if (gamepad1.b) {
                // run motor to an target position
                runToPosition(motors[currentMotor].target);
                while (gamepad1.b) sleep(10);

            } else if (gamepad1.left_trigger > 0) {
                // manually run the motor backwards
                motor.setPower(-speed);
                while (gamepad1.left_trigger > 0) {
                    setDisplayPosition(positionMsg);
                    telemetry.update();
                    Logger.message("y %f", gamepad1.left_trigger);
                    sleep(100);
                }
                motor.setPower(0);

            } else if (gamepad1.right_trigger > 0) {
                // manually run the motor forward
                motor.setPower(speed);
                while (gamepad1.right_trigger > 0) {
                    setDisplayPosition(positionMsg);
                    telemetry.update();
                    sleep(100);
                }
                motor.setPower(0);

            } else if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
                incrementFine.reset();
                incrementCoarse.reset();
                while (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0 ) {
                    if (gamepad1.left_stick_x > 0)
                        motors[currentMotor].home += (int)incrementFine.get();
                    else if (gamepad1.left_stick_x < 0)
                        motors[currentMotor].home -= (int)incrementFine.get();
                    else if (gamepad1.left_stick_y < 0)
                        motors[currentMotor].home += (int)incrementCoarse.get();
                    else if (gamepad1.left_stick_y > 0)
                        motors[currentMotor].home -= (int)incrementCoarse.get();
                    setDisplayHome(homeMsg);
                    telemetry.update();
                }

            } else if (gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0) {
                incrementFine.reset();
                incrementCoarse.reset();
                while (gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0 ) {
                    if (gamepad1.right_stick_x > 0)
                        motors[currentMotor].target += (int)incrementFine.get();
                    else if (gamepad1.right_stick_x < 0)
                        motors[currentMotor].target -= (int)incrementFine.get();
                    else if (gamepad1.right_stick_y < 0)
                        motors[currentMotor].target += (int)incrementCoarse.get();
                    else if (gamepad1.right_stick_y > 0)
                        motors[currentMotor].target -= (int)incrementCoarse.get();
                    setDisplayTarget(targetMsg);
                    telemetry.update();
                }

            } else if (gamepad1.left_bumper) {
                // increase motor speed
                speedIncrement.reset();
                while (gamepad1.left_bumper) {
                    speed = Math.max(speed - speedIncrement.get(), 0);
                    setDisplaySpeed(speedMsg);
                    telemetry.update();
                }
                motors[currentMotor].speed = speed;

            } else if (gamepad1.right_bumper) {
                // decrease the motor speed
                speedIncrement.reset();
                while (gamepad1.right_bumper) {
                    speed = Math.min(speed + speedIncrement.get(), 0.95);
                    setDisplaySpeed(speedMsg);
                    telemetry.update();
                }
                motors[currentMotor].speed = speed;

            } else if (gamepad1.dpad_left) {
                // select the next motor
                selectMotor(MOTOR_SELECT.PREVIOUS);
                setDisplaySpeed(speedMsg);
                setDisplayDirection(directionMsg);
                while (gamepad1.dpad_left){
                    sleep(10);
                }
                setDisplayName (motorNameMsg);

            } else if (gamepad1.dpad_right) {
                // select the previous motor
                selectMotor(MOTOR_SELECT.NEXT);
                setDisplaySpeed(speedMsg);
                setDisplayDirection(directionMsg);
                while (gamepad1.dpad_right){
                    sleep(10);
                }
                setDisplayName (motorNameMsg);

            } else if (gamepad1.dpad_up) {
                // change the direction of the motor
                if (motor.getDirection() == DcMotor.Direction.FORWARD)
                    motor.setDirection(DcMotor.Direction.REVERSE);
                else
                    motor.setDirection(DcMotor.Direction.FORWARD);
                setDisplayDirection(directionMsg);
                while (gamepad1.dpad_up) {
                    sleep(10);
                }

            } else if (gamepad1.dpad_down) {
                // change motor breaking
                if (motor.getPowerFloat())
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                else
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                setDisplayBraking(directionMsg);
                while (gamepad1.dpad_up) {
                    sleep(10);
                }

            } else if (gamepad1.left_stick_button) {
                // zero encoder
                DcMotor.RunMode mode = motor.getMode();
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor.setMode(mode);

            } else if (gamepad1.back) {
                // exit opmode without saving calibration data
                save = false;
                requestOpModeStop();
                break;
            }

            setDisplayPosition(positionMsg);
            setDisplayHome(homeMsg);
            setDisplayTarget(targetMsg);
            telemetry.update();
        }

        if (save) {
            writeSettings();
        }
    }

    void setDisplayName (Telemetry.Item item) {
        item.setValue("%s  (port: %d)", motors[currentMotor].name, motor.getPortNumber());
    }

    private void setDisplayPosition (Telemetry.Item item) {
        item.setValue( "%d", motors[currentMotor].motor.getCurrentPosition());
    }

    private void setDisplayHome (Telemetry.Item item) {
        item.setValue( "%d", motors[currentMotor].home);
    }

    private void setDisplayTarget (Telemetry.Item item) {
        item.setValue( "%d", motors[currentMotor].target);
    }

    private void setDisplayDirection (Telemetry.Item item) {
        item.setValue("%s", motors[currentMotor].motor.getDirection());
    }

    private void setDisplayBraking (Telemetry.Item item) {
        item.setValue("%s", motors[currentMotor].motor.getZeroPowerBehavior());
    }

    private void setDisplaySpeed (Telemetry.Item item) {
        item.setValue( "%5.3f", speed);
    }

    private void runToPosition(int position) {

        DcMotor.RunMode mode = motor.getMode();
        Logger.message("run from %d to %d", motor.getCurrentPosition(), position);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(speed);
        while (opModeIsActive()) {
            Logger.message("position %d", motor.getCurrentPosition());
            if (! motor.isBusy())
                break;
        }
        if (holdPower) {
            runtime.reset();
            while (runtime.seconds() > 5 && opModeIsActive())
                sleep(100);
        }
        motor.setPower(0);
        motor.setMode(mode);
    }

     // Build a list of motors sorted by port number
    private void getMotors(){

        motorCount = 0;
        SortedSet<String> names = hardwareMap.getAllNames(DcMotor.class);
        for (String name: names){
            DcMotor m = hardwareMap.get(DcMotor.class, name);
            
            motors[motorCount] = new MotorInfo();
            motors[motorCount].name = name;
            motors[motorCount].motor = m;
            motors[motorCount].home = 0;
            motors[motorCount].target = 0;
            motors[motorCount].speed = speed;
            motorCount++;
        }

        // Sort by port numbers
        Arrays.sort(motors, 0, motorCount);

        for (int i = 0; i < motors.length; i++) {
            if (motors[i] != null) {
                if (motor == null) {
                    motor = motors[i].motor;
                    currentMotor = i;
                }
                Logger.message("motor name %s port %d", motors[i].name, motors[i].motor.getPortNumber());
            }
        }
    }

    private void selectMotor (MOTOR_SELECT select){
        int index;
        for (int i = 1; i <= motors.length; i++) {
            if (select == MOTOR_SELECT.NEXT)
                index = (currentMotor + i) %  motors.length;
            else
                index = (currentMotor + motors.length - i) %  motors.length;

            if (motors[index] != null){
                motor = motors[index].motor;
                speed = motors[index].speed;
                currentMotor = index;
                break;
            }
        }
    }

    private String getSettingsPath () {

        String settingsFile = this.getClass().getSimpleName();
        return String.format("%s%s.%s", settingsDir, settingsFile, "txt");
    }

    private void makeSettingsDirectory() {

        // create the directory if it does not exist.
        File directory = new File(settingsDir);
        boolean exist = directory.exists();

        if (! exist ) {
            boolean created = directory.mkdir();
            if (! created) {
                Logger.warning("could not create directory %s", settingsDir);
            }
        }
    }

    public void writeSettings ()  {
        try {

            Settings settings = new Settings();
            settings.motors = new ArrayList<>();

            settings.currentMotor = motors[currentMotor].name;

            for (MotorInfo motor : motors) {
                if (motor != null && !motor.name.isEmpty()) {
                    MotorSettings motorSettings = new MotorSettings();
                    motorSettings.name = motor.name;
                    motorSettings.home = motor.home;
                    motorSettings.target = motor.target;
                    motorSettings.speed = motor.speed;
                    motorSettings.direction = motor.motor.getDirection();
                    settings.motors.add(motorSettings);
                    Logger.message("%s  home: %5d  target: %5d", motor.name, motor.home, motor.target);
                }
            }

            makeSettingsDirectory();
            Gson gson = new GsonBuilder().setPrettyPrinting().create();
            FileWriter f = new FileWriter(getSettingsPath(), false);

            gson.toJson(settings, f);
            f.flush();
            f.close();

        } catch (Exception e) {
            Logger.error(e, "Error writing settings");
        }
    }

    private void readSettings() {

        try {
            // read the settings file
            BufferedReader br = new BufferedReader(new FileReader(getSettingsPath()));
            Settings settings = new Gson().fromJson(br, Settings.class);
            if (settings != null) {

                // set the current motor
                for (int i = 0; i < motorCount; i++) {
                    MotorInfo motor = motors[i];
                    if (motor.name.equals(settings.currentMotor)) {
                        currentMotor = i;
                    }

                    // copy the settings for each motor
                    if (!motor.name.isEmpty()) {
                        for (MotorSettings motorSettings: settings.motors) {
                            if (motor.name.equals(motorSettings.name)) {
                                motor.home = motorSettings.home;
                                motor.target = motorSettings.target;
                                motor.speed = motorSettings.speed;
                                motor.motor.setDirection(motorSettings.direction);
                                Logger.message("%s  home: %5d  target: %5d", motor.name, motor.home, motor.target);
                            }
                        }
                    }
                }
            }

        } catch (Exception e) {
            Logger.error(e, "Error reading settings");
        }
    }
}

