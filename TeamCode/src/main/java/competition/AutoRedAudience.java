package competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import common.Auto;
import common.Logger;

@Autonomous(name="Red Audience", group="Competition")

public class AutoRedAudience extends LinearOpMode {

    @Override
    public void runOpMode() {

        try {
            long delay = 0;
            double launcherSpeed = 29;
            Auto.Order[] order = { Auto.Order.BOTTOM };

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
