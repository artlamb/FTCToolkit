package test;

import android.annotation.SuppressLint;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import common.Drive;
import common.DriveControl;
import common.Logger;
import utils.Pose;

@TeleOp(name="DecelerationTest", group="Test")
@SuppressLint("DefaultLocale")
//@Disabled

@com.acmerobotics.dashboard.config.Config

public class DecelerationTest extends LinearOpMode {
    public static boolean turn = false;

    Drive drive  = null;
    DriveControl driveControl;

    @Override
    public void runOpMode() {
        try {
            drive = new Drive(this);
            drive.start();
            driveControl = new DriveControl(this, drive);
            driveControl.resetIMU();
            driveControl.start();

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            telemetry.addLine("Press start");
            telemetry.update();

            waitForStart();

            if (turn) {
                turnDecelerationTest();
            } else {
                driveDecelerationTest();
            }

        } catch (Exception e) {
            Logger.error(e, "Error");
        }
    }

    private void driveDecelerationTest() {

        driveControl.reset();
        Pose start = driveControl.getPose();

        for (double percent = 0.05; percent < 0.75; percent += 0.05) {

            driveControl.setPose(new Pose());
            driveControl.decelerationTest(percent, 0, turn);

            sleep(2000);

            Logger.setErrorLevel(Log.WARN);
            driveControl.moveToPose(start, 3000);
            driveControl.waitUntilNotMoving(5000);
            Logger.setErrorLevel(Log.VERBOSE);
        }
    }

    private void turnDecelerationTest() {

        driveControl.reset();

        for (double percent = 0.01; percent <= 0.1; percent += 0.01) {

            driveControl.setPose(new Pose());
            driveControl.decelerationTest(percent, 0, turn);

            sleep(2000);
        }
    }
}
