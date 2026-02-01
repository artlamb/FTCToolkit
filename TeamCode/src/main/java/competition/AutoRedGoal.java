package competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import common.Auto;
import common.Logger;

@Autonomous(name="Red Goal", group="Competition")

public class AutoRedGoal extends LinearOpMode {

    @Override
    public void runOpMode() {

        try {
            int timesToShoot = 3;
            long delay = 0;
            double launcherSpeed = 26;
            Auto.Order[] order = { Auto.Order.TOP, Auto.Order.MIDDLE, Auto.Order.BOTTOM };

            Auto auto = new common.Auto(this,
                    Auto.Alliance.RED,
                    Auto.StartPosition.GOAL,
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
