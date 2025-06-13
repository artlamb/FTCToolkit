package test;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import common.Auto;
import common.Logger;
import utils.PoseData;

@TeleOp(name="Auto Test", group="Test")
@SuppressLint("DefaultLocale")
@com.acmerobotics.dashboard.config.Config

public class AutoTest extends LinearOpMode {

    public static PoseData START = new PoseData(0, 0, 0);
    public static PoseData WAYPOINT_1 = new PoseData(20, 0, 0);
    public static PoseData WAYPOINT_2 = new PoseData(20, 400, 90);
    public static PoseData PARK = new PoseData(0, 0, 00);

    Auto auto;

    @Override
    public void runOpMode() {

        try {
            initialize();

            telemetry.addLine("Press start");
            telemetry.update();
            waitForStart();

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
        auto.addPath(Auto.PathState.WAYPOINT_2, WAYPOINT_2.x, WAYPOINT_2.y, WAYPOINT_2.h);
        auto.addPath(Auto.PathState.PARK, PARK.x, PARK.y, PARK.h);
    }
}
