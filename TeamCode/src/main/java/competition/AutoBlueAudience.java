package competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import common.Auto;
import common.Logger;

@Autonomous(name="Blue Audience", group="Competition")

public class AutoBlueAudience extends LinearOpMode {

    @Override
    public void runOpMode() {

        int timesToShoot = 1;
        long delay = 0;
        double launcherSpeed = 29;
        Auto.Order[] order = { Auto.Order.BOTTOM };

        try {
            Auto auto = new common.Auto(this,
                    Auto.Alliance.BLUE,
                    Auto.StartPosition.AUDIENCE,
                    order,
                    launcherSpeed,
                    delay,
                    timesToShoot);

            auto.runAuto();

        } catch (Exception e) {
            Logger.error(e, "Exception");
            throw e;
        }
    }
}
