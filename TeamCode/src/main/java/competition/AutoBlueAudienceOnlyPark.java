package competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import common.Auto;
import common.Logger;

@Autonomous(name="Blue Audience Only Park", group="Competition")

public class AutoBlueAudienceOnlyPark extends LinearOpMode {

    @Override
    public void runOpMode() {

        try {
            int timesToShoot = 0;
            long delay = 0;
            double launcherSpeed = 29;
            Auto.Order[] order = {  };

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
