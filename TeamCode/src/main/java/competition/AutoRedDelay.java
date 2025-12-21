package competition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import common.Auto;
import common.Logger;
import utils.PoseData;

@Autonomous(name="Auto Red Delay", group="Competition")
@com.acmerobotics.dashboard.config.Config

public class AutoRedDelay extends LinearOpMode {

    public static double LAUNCHER_SPEED = 27;

    public static PoseData START =      new PoseData(12.25, -62,  90.0);
    public static PoseData WAYPOINT_1 = new PoseData(12.25, 12,   45.0);
    public static PoseData PARK =       new PoseData(12.25, -36,  180.0);
    public static long     WAIT_TIME = 10;

    Auto auto;

    @Override
    public void runOpMode() {

        try {
            initialize();

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            auto.runAuto(WAIT_TIME);

        } catch (Exception e) {
            Logger.error(e, "Exception");
            throw e;
        }
    }

    private void initialize() {
        auto = new Auto(this);

        auto.addPath(Auto.PathState.START, START.x, START.y, START.h);
        auto.addPath(Auto.PathState.SHOOT, WAYPOINT_1.x, WAYPOINT_1.y, WAYPOINT_1.h);
        auto.addPath(Auto.PathState.PARK, PARK.x, PARK.y, PARK.h);

        auto.setStartPose(START.x, START.y, START.h);
        auto.setLauncherSpeed(LAUNCHER_SPEED);
        auto.drawPaths();
    }
}
