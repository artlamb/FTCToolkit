package test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import common.Logger;
import common.Settings;

@TeleOp(name="SettingsTest", group="Test")
@Disabled
@SuppressWarnings("unused")

public class SettingsTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Press start");
        telemetry.update();

        waitForStart();



        Logger.message("drive factor: %f  strafe factor: %f turn factor: %f",
                Settings.getDriveFactor(),
                Settings.getStrafeFactor(),
                Settings.getTurnFactor());

        Settings.writeSettings();

        while (opModeIsActive()) {
            sleep(10);
        }
    }
}

