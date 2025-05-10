package test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import common.Config;
import common.Drive;
import common.DriveControl;
import common.DriveGamepad;
import common.Logger;
import utils.Pose;

@Disabled
@TeleOp(name="Distance Test", group="Test")

public class DistanceTest extends LinearOpMode {

    DriveControl driveControl;
    DriveGamepad driveGamepad;
    DistanceSensor leftFrontSensor;
    DistanceSensor rightFrontSensor;

    @Override
    public void runOpMode() {

        driveControl = new DriveControl(this, new Drive(this));
        driveGamepad = new DriveGamepad(this, driveControl);
        driveControl.start();
        driveGamepad.start();

        initDistanceSensors();

        telemetry.addLine("Press start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                alignInCorner();
                while (gamepad1.a) sleep(10);
            }
            if (gamepad1.b) {
                sensorNoiseTest();
                while (gamepad1.b) sleep(10);
            }

            telemetry.addData("distance",  "%5.2f  %5f.2",
                    leftFrontSensor.getDistance(DistanceUnit.INCH),
                    rightFrontSensor.getDistance(DistanceUnit.INCH));

            telemetry.update();
            sleep(50);
        }
    }

    private void initDistanceSensors() {

        try {
            leftFrontSensor = hardwareMap.get(DistanceSensor.class, Config.LEFT_FRONT_SENSOR);
        } catch (Exception e) {
            Logger.error(e, "Left distance sensor not found");
        }

        try {
            rightFrontSensor = hardwareMap.get(DistanceSensor.class, Config.RIGHT_FRONT_SENSOR);
        } catch (Exception e) {
            Logger.error(e, "Right distance sensor not found");
        }
    }

    private void alignInCorner() {

        while (opModeIsActive()) {
            if (gamepad1.x) {
                driveControl.setPose(new Pose(0, 0, Math.toRadians(0)));
                driveControl.alignInCorner();
                while (gamepad1.x) sleep(100);
            }
        }
    }

    private void sensorNoiseTest() {
        // calculate the signal noise of a of the distance sensors.
        initDistanceSensors();

        double minX = 1000000;
        double minY = 1000000;
        double maxX = 0;
        double maxY = 0;
        double x;
        double y;
        double avgX = 0;
        double avgY = 0;
        double samples = 5;

        Logger.message("get distance");
        for (int i = 0; i < samples; i++) {
            avgX += leftFrontSensor.getDistance(DistanceUnit.INCH);
            avgY += rightFrontSensor.getDistance(DistanceUnit.INCH);
        }
        avgX /= samples;
        avgY /= samples;
        Logger.message("x %6.1f  y %6.1f", avgX, avgY);

        while (opModeIsActive()) {
            x = leftFrontSensor.getDistance(DistanceUnit.INCH);
            y = rightFrontSensor.getDistance(DistanceUnit.INCH);
            minX = Math.min(minX, x);
            minY = Math.min(minY, y);
            maxX = Math.max(maxX, x);
            maxY = Math.max(maxY, y);

            telemetry.addData("distance", "x %6.1f  y %6.1f", x, y);
            telemetry.addData("min", "x %6.1f  y %6.1f", minX, minY);
            telemetry.addData("max", "x %6.1f  y %6.1f", maxX, maxY);
            telemetry.update();
            sleep(100);
        }
    }
}
