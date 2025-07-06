package test;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import common.Logger;
import common.Robot;
import utils.Dashboard;
import utils.Pose;

@Disabled
@TeleOp(name="Dashboard Test", group="Test")
@SuppressLint("DefaultLocale")

public class DashboardTest extends LinearOpMode {

    Robot robot;
    Dashboard dashboard;

    @Override
    public void runOpMode() {
        try {
            robot = new Robot(this);
            dashboard = new Dashboard();

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            telemetry.addLine("Press start");
            telemetry.update();

            //drawOnDashboard(new Pose2D(DistanceUnit.INCH,-(24), 23.5, AngleUnit.RADIANS, Math.PI));

            dashboard.setPose(new Pose(2, 0.75, 0, 0, Math.PI/2));
            dashboard.addWaypoint(new Pose(0, 0, 0, 0, 0));
            dashboard.addWaypoint(new Pose(1, 0, 0, 0, 0));
            dashboard.addWaypoint(new Pose(-2, -2, -10, -10, 0));

            dashboard.drawField();

            waitForStart();

            while (opModeIsActive()) {
                sleep(500);
            }

        } catch (Exception e) {
            Logger.error(e, "Error");
        }
    }

}
