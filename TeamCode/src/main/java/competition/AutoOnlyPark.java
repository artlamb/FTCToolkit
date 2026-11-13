package competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import common.Auto;
import common.Logger;

@Autonomous(name="Auto Only Park", group="Competition")

public class AutoOnlyPark extends LinearOpMode {

    @Override
    public void runOpMode() {

        try {
            int timesToShoot = 0;
            long delay = 0;
            double launcherSpeed = 29;
            Auto.Order[] order = {  };

            Auto auto = new Auto(this,
                    Auto.Alliance.RED,
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
