package competition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import common.Auto;
import common.Logger;
import utils.PoseData;

@Autonomous(name="Auto Red Goal", group="Competition")
@com.acmerobotics.dashboard.config.Config

public class AutoRedGoal extends LinearOpMode {

    public static double LAUNCHER_SPEED = 28;

    public static PoseData START =      new PoseData(47.0, 47.0,  45.0);
    public static PoseData WAYPOINT_1 = new PoseData(12.5,  12.5, 45.0);
    public static PoseData PARK =       new PoseData(12.5, -12.5, 1800.0);

    Auto auto;

    @Override
    public void runOpMode() {

        try {
            initialize();

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            auto.runAuto();

        } catch (Exception e) {
            Logger.error(e, "Exception");
            throw e;
        }
    }

    private void initialize() {
        auto = new Auto(this);

        auto.addPath(Auto.PathState.START, START.x, START.y, START.h);
        auto.addPath(Auto.PathState.WAYPOINT_1, WAYPOINT_1.x, WAYPOINT_1.y, WAYPOINT_1.h);
        auto.addPath(Auto.PathState.PARK, PARK.x, PARK.y, PARK.h);

        auto.setStartPose(START.x, START.y, START.h);
        auto.setLauncherSpeed(LAUNCHER_SPEED);
        auto.drawPaths();
    }
}
