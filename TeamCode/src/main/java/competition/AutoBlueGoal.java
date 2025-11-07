package competition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import common.Auto;
import common.Logger;
import utils.PoseData;

@Autonomous(name="Auto Blue Goal", group="Competition")
@com.acmerobotics.dashboard.config.Config

public class AutoBlueGoal extends LinearOpMode {

    public static PoseData START =      new PoseData(-47.0, 47.0,135.0);
    public static PoseData WAYPOINT_1 = new PoseData(-12, 12,    135.0);
    public static PoseData PARK =       new PoseData(-47, -47,   0.0);

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

        auto.setStartPose(START.x, START.y, START.h);
        auto.addPath(Auto.PathState.WAYPOINT_1, WAYPOINT_1.x, WAYPOINT_1.y, WAYPOINT_1.h);
        auto.addPath(Auto.PathState.PARK, PARK.x, PARK.y, PARK.h);
    }
}
