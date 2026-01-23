package competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import common.Auto;
import common.Logger;

@Autonomous(name="Blue Goal", group="Competition")

public class AutoBlueGoal extends LinearOpMode {

    @Override
    public void runOpMode() {

        try {
            long delay = 0;
            double launcherSpeed = 26;
            Auto.Order[] order = { Auto.Order.TOP, Auto.Order.MIDDLE };

            Auto auto = new common.Auto(this,
                    Auto.Alliance.BLUE,
                    Auto.StartPosition.GOAL,
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
