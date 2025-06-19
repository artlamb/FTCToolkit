package test;

import android.annotation.SuppressLint;

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

public class DecelerationTest extends LinearOpMode {

    Drive drive  = null;
    DriveControl driveControl;
    DcMotorEx odometer;

    @Override
    public void runOpMode() {
        try {
            drive = new Drive(this);
            drive.start();
            driveControl = new DriveControl(this, drive);
            driveControl.resetIMU();
            driveControl.start();

            odometer = drive.leftFrontDrive;

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            telemetry.addLine("Press start");
            telemetry.update();

            waitForStart();

            decelerationTest();

        } catch (Exception e) {
            Logger.error(e, "Error");
        }

    }

    private void decelerationTest() {

        Pose start = driveControl.getPose();

        boolean turn = true;
        for (double percent = 0.1; percent <= 0.7; percent += 0.1) {

            driveControl.setPose(new Pose());
            driveControl.decelerationTest(percent, 0, true);

            sleep(2000);

            if (! turn) {
                driveControl.moveToPose(start, 3000);
                driveControl.waitUntilNotMoving();
            }
        }
    }
}
