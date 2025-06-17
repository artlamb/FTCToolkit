package tools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.Locale;
import java.util.SortedSet;

import common.Logger;
import utils.Increment;

/**
 * This OpMode calibrate any of the robot servos.
 */

@TeleOp(name="Calibrate Servo", group="Tools")

public class CalibrateServo extends LinearOpMode {

    private final String calibrationPath = "/temp/servoCalibration.txt";

    private enum SERVO_SELECT { PREVIOUS, NEXT }

    private Servo servo = null;

    private static class ServoInfo {
        String  name;
        Servo   servo;
        double  home;
        double  target;
    }
    ServoInfo[] servos = new ServoInfo[12];
    int servoCount;
    int currentServo;

    @Override
    public void runOpMode() {

        Increment incrementFine   = new Increment(0.001, 0.002, 0.004);
        Increment incrementCoarse = new Increment(0.010, 0.020, 0.040);

        getServos();
        readCalibration();

        telemetry.addLine("Press start");
        telemetry.update();
        telemetry.setAutoClear(false);

        waitForStart();

        Telemetry.Item servoNameMsg =  telemetry.addData("Servo name", 0);
        Telemetry.Item directionMsg = telemetry.addData("Servo direction", 0);
        Telemetry.Item positionMsg = telemetry.addData("Servo position", 0);
        Telemetry.Item homeMsg = telemetry.addData("Home position", 0);
        Telemetry.Item targetMsg = telemetry.addData("Target position", 0);

        setDisplayName(servoNameMsg);
        setDisplayDirection(directionMsg);

        telemetry.addData("\nServo Calibration Controls", "\n" +
                "  left stick - increase/decrease home position\n" +
                "  right stick - increase/decrease target position\n" +
                "  x - run to home position\n" +
                "  b - run servo to target position\n" +
                "  y - set home position to current position\n" +
                "  a - set target position to current position\n" +
                "  dpad left - select previous servo\n" +
                "  dpad right - select next servo\n" +
                "  left trigger - run servo backwards\n" +
                "  right trigger - run the servo forward\n" +
                "\n");

        telemetry.update();

        boolean save = true;

        while (opModeIsActive()) {

            if (gamepad1.a) {
                // set target position to current position
                servos[currentServo].target = servo.getPosition();

            } else if (gamepad1.y) {
                // set the home position to the current position
                servos[currentServo].home = servo.getPosition();

            } else if (gamepad1.x) {
                // run to home position
                servo.setPosition(servos[currentServo].home);

            } else if (gamepad1.b) {
                // run servo to an target position
                servo.setPosition(servos[currentServo].target);

            } else if (gamepad1.left_trigger > 0) {
                // manually run the servo backwards
                while (gamepad1.left_trigger > 0) {
                    double position = servo.getPosition();
                    if (Double.isNaN(position)) {
                        Logger.message("Servo position not set");
                        break;
                    } else {
                        position -= 0.001;
                    }
                    servo.setPosition(position);
                    setDisplayPosition(positionMsg);
                    telemetry.update();
                    sleep(100);
                }

            } else if (gamepad1.right_trigger > 0) {
                // manually run the servo forward
                while (gamepad1.right_trigger > 0) {
                    double position = servo.getPosition();
                    if (Double.isNaN(position)) {
                        Logger.message("Servo position not set");
                        break;
                    } else {
                        position += 0.001;
                    }
                    servo.setPosition(position);
                    setDisplayPosition(positionMsg);
                    telemetry.update();
                    sleep(100);
                }

            } else if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
                incrementFine.reset();
                incrementCoarse.reset();
                while (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0 ) {
                    if (gamepad1.left_stick_x > 0)
                        servos[currentServo].home = Math.min(1, servos[currentServo].home + incrementFine.get());
                    else if (gamepad1.left_stick_x < 0)
                        servos[currentServo].home = Math.max(0, servos[currentServo].home - incrementFine.get());
                    else if (gamepad1.left_stick_y < 0)
                        servos[currentServo].home = Math.min(1, servos[currentServo].home + incrementCoarse.get());
                    else if (gamepad1.left_stick_y > 0)
                        servos[currentServo].home = Math.max(0, servos[currentServo].home - incrementCoarse.get());
                    setDisplayHome(homeMsg);
                    telemetry.update();
                }

            } else if (gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0) {
                incrementFine.reset();
                incrementCoarse.reset();
                while (gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0 ) {
                    if (gamepad1.right_stick_x > 0)
                        servos[currentServo].target = Math.min(1, servos[currentServo].target + incrementFine.get());
                    else if (gamepad1.right_stick_x < 0)
                        servos[currentServo].target = Math.max(0, servos[currentServo].target - incrementFine.get());
                    else if (gamepad1.right_stick_y < 0)
                        servos[currentServo].target = Math.min(1, servos[currentServo].target + incrementCoarse.get());
                    else if (gamepad1.right_stick_y > 0)
                        servos[currentServo].target = Math.max(0, servos[currentServo].target - incrementCoarse.get());
                    setDisplayTarget(targetMsg);
                    telemetry.update();
                }

            } else if (gamepad1.dpad_left) {
                // select the next motor
                selectServo(SERVO_SELECT.PREVIOUS);
                while (gamepad1.dpad_left){
                    sleep(10);
                }
                setDisplayName(servoNameMsg);

            } else if (gamepad1.dpad_right) {
                // select the previous servo
                selectServo(SERVO_SELECT.NEXT);
                while (gamepad1.dpad_right) {
                    sleep(10);
                }
                setDisplayName(servoNameMsg);

            } else if (gamepad1.dpad_up) {
                // change the direction of the servo
                if (servo.getDirection() == Servo.Direction.FORWARD)
                    servo.setDirection(Servo.Direction.REVERSE);
                else
                    servo.setDirection(Servo.Direction.FORWARD);
                setDisplayDirection(directionMsg);
                while (gamepad1.dpad_up) sleep(10);

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
            writeCalibration();
        }
    }

    private void setDisplayName (Telemetry.Item item) {
        item.setValue("%s  (port: %d)", servos[currentServo].name, servos[currentServo].servo.getPortNumber());
    }

    private void setDisplayDirection (Telemetry.Item item) {
        item.setValue("%s", servos[currentServo].servo.getDirection());
    }

    private void setDisplayPosition (Telemetry.Item item) {
        item.setValue( "%5.3f", servos[currentServo].servo.getPosition());
    }

    private void setDisplayHome (Telemetry.Item item) {
        item.setValue("%5.3f", servos[currentServo].home);
    }

    private void setDisplayTarget (Telemetry.Item item) {
        item.setValue("%5.3f", servos[currentServo].target);
    }

    /**
     * Build a list of servos sorted by port number
     */
    private void getServos(){

        servoCount = 0;
        SortedSet<String> names = hardwareMap.getAllNames(Servo.class);
        for (String name: names){
            Servo s = hardwareMap.get(Servo.class, name);
            servos[servoCount] = new ServoInfo();
            servos[servoCount].name = name;
            servos[servoCount].servo = s;
            servos[servoCount].home = 0.5;
            servos[servoCount].target = 0.5;
            servoCount++;
        }

        for (int i = 0; i < servos.length; i++) {
            if (servos[i] != null) {
                if (servo == null) {
                    servo = servos[i].servo;
                    currentServo = i;
                }
                Logger.message("servo name %s port %d", servos[i].name, servos[i].servo.getPortNumber());
            }
        }
    }

    private void selectServo (SERVO_SELECT select){
        int index;
        for (int i = 1; i <= servos.length; i++) {
            if (select == SERVO_SELECT.NEXT)
                index = (currentServo + i) %  servos.length;
            else
                index = (currentServo + servos.length - i) %  servos.length;

            if (servos[index] != null){
                servo = servos[index].servo;
                currentServo = index;
                break;
            }
        }
    }

    // Write the calibration values to a comma separated text file.
    private void writeCalibration() {

        try {
            String path = String.format("%s%s", AppUtil.FIRST_FOLDER.getAbsolutePath(), calibrationPath);
            Logger.message ("Writing calibration data to %s", path);

            FileWriter f = new FileWriter(path);

            for (ServoInfo servosInfo : servos) {
                if (servosInfo != null) {
                    String str = String.format(Locale.ENGLISH, "%s,%f,%f\n", servosInfo.name, servosInfo.home, servosInfo.target);
                    f.write(str);
                    Logger.message(str);
                }
            }

            f.flush();
            f.close();

        } catch(Exception e) {
            Logger.error(e, "");
        }
    }

    // Read the calibration values from a comma separated text file.
    private void readCalibration() {
        try {
            String path = String.format("%s%s", AppUtil.FIRST_FOLDER.getAbsolutePath(), calibrationPath);
            Logger.message ("Reading calibration data from %s", path);

            BufferedReader reader;
            try {
                reader = new BufferedReader(new FileReader(path));
            } catch (java.io.FileNotFoundException ex) {
                Logger.message("Servo calibration file not found");
                return;
            }

            String line = reader.readLine();
            while (line != null) {
                // parse the line read
                int end = line.indexOf(',');
                if (end > 0) {
                    String name = line.substring(0, end);
                    int start = end + 1;
                    end = line.indexOf(',', start);
                    if (end > 0) {
                        String homeStr =  line.substring(start, end);
                        start = end + 1;
                        String targetStr = line.substring(start);
                        for (ServoInfo servosInfo : servos) {
                            if (servosInfo != null && name.equals(servosInfo.name)) {
                                servosInfo.home = Double.parseDouble(homeStr);
                                servosInfo.target = Double.parseDouble(targetStr);
                                String str = String.format(Locale.ENGLISH, "%s,%f,%f\n", servosInfo.name, servosInfo.home, servosInfo.target);
                                Logger.message(str);
                                break;
                            }
                        }
                    }
                }

                line = reader.readLine();
            }

            reader.close();

        } catch(Exception e) {
            Logger.error(e, "");
        }
    }
}

