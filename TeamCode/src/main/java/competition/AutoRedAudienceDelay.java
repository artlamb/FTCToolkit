package competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import common.Auto;
import common.Logger;

@Disabled
@Autonomous(name="Auto Red Delay", group="Competition")

public class AutoRedAudienceDelay extends LinearOpMode {

    @Override
    public void runOpMode() {

        try {
            long delay = 10;
            double launcherSpeed = 27;
            Auto.Order[] order = { Auto.Order.TOP, Auto.Order.MIDDLE };

            Auto auto = new common.Auto(this,
                    Auto.Alliance.RED,
                    Auto.StartPosition.AUDIENCE,
                    order,
                    launcherSpeed,
                    delay);

            auto.runAuto();

        } catch (Exception e) {
            Logger.error(e, "Exception");
            throw e;
        }
    }
}
