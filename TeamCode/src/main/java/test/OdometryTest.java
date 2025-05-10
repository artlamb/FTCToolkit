package test;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import common.Config;
import common.Logger;
import common.Robot;
import drivers.GoBildaPinpointDriver;

@TeleOp(name="OdometryTest", group="Test")
@SuppressLint("DefaultLocale")

public class OdometryTest extends LinearOpMode {

    Robot robot;
    GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {
        try {
            robot = new Robot(this);
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Config.PINPOINT);
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.RADIANS, 0));

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            telemetry.addLine("Press start");
            telemetry.update();

            waitForStart();

            drawOnDashboard();

            while (opModeIsActive()) {
                displayPose(true);
                sleep(500);
            }

        } catch (Exception e) {
            Logger.error(e, "Error");
        }
    }

    private void displayPose(boolean telemetryOnly) {

        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();

        String str1 = String.format("x %5.1f  y %5.1f  heading %5.1f",
                pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.DEGREES));
        String str2 = String.format("x %d  y %d", pinpoint.getEncoderX(), pinpoint.getEncoderY());

        telemetry.addData("pinpoint pose", str1);
        telemetry.addData("encoders", str2);
        telemetry.update();

        if (! telemetryOnly) {
            Logger.message("pose: %s  encoders %s", str1, str2);
        }
    }

    private void drawOnDashboard() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        if (! dashboard.isEnabled()) {
            Logger.warning("dashboard is not enabled");
            return;
        }

        double robotWidth = 18;
        double robotHeight = 18;

        TelemetryPacket packet = new TelemetryPacket(false);
        packet.fieldOverlay()
                //.setScale(0.75, 0.75)
                //.setTranslation(10, 10)
                .setAlpha(1.0)
                .setRotation(-Math.PI/2)
                //.drawImage("/images/robot1.png", -(robotWidth/2), (robotHeight/2), robotWidth, robotHeight, -Math.PI/2, 0, 0, false)
                //.drawImage("/images/robot1.png", -robotCenter, robotCenter, robotWidth, robotHeight, 0, 0, 0, false)
                .drawImage("/images/robot1.png", 0, 0, robotWidth, robotHeight, -Math.PI/4, 0, robotHeight, false)
                .drawGrid(0, 0, 144, 144, 7, 7)
                .setStrokeWidth(1)
                .setStroke("green")
                .strokeLine(0, 0, 0, 24) //y axis
                .setStroke("red")
                .strokeLine(0, 0, 24, 0); //x axis

        //packet.fieldOverlay().fillRect(10,10,100,100);
        dashboard.sendTelemetryPacket(packet);
        sleep(20);
    }
}
