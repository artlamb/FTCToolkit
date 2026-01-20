package competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import common.Auto;
import common.Logger;

@Autonomous(name="Blue Audience", group="Competition")

public class AutoBlueAudience extends LinearOpMode {

    @Override
    public void runOpMode() {

        long delay = 0;
        double launcherSpeed = 27;
        Auto.Order[] order = { Auto.Order.TOP, Auto.Order.MIDDLE };

        try {
            Auto auto = new common.Auto(this,
                    Auto.Alliance.BLUE,
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
