package test;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import common.Limelight;
import common.Logger;

@TeleOp(name="AprilTagTest", group="Test")
@SuppressLint("DefaultLocale")
@Disabled

public class AprilTagTest extends LinearOpMode {

    //Robot robot;
    //GoBildaPinpointDriver pinpoint;
    Limelight limelight;

    @Override
    public void runOpMode() {
        try {
            //robot = new Robot(this);
            limelight = new Limelight(this);
            limelight.setPipeline(Limelight.Pipeline.APRIL_TAG);
            //pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Config.PINPOINT);
            //pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.RADIANS, 0));

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            telemetry.addLine("Press start");
            telemetry.update();

            waitForStart();

            while (opModeIsActive()) {
                displayPosition(false);
                sleep(500);
            }

        } catch (Exception e) {
            Logger.error(e, "Error");
        }
    }

    private void displayPosition(boolean telemetryOnly) {

        double tx = limelight.GetTx();

        String str1 = String.format("tx %5.1f", tx);

        telemetry.addData("april tag", str1);
        telemetry.update();

        if (! telemetryOnly) {
            Logger.message("april tab: %s", str1);
        }
    }
}
