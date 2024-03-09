package frc.robot.util;

import frc.robot.RobotContainer;
import frc.robot.RobotContainerSmudge;
import frc.robot.config.RobotIdentity;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstantsSmudge;

public interface RobotConstants {

    static TunerConstantsSmudge getRobotConstants(RobotIdentity robot) {
        switch (robot) {
            case SMUDGE_2024:
                return new TunerConstantsSmudge();
            case SMIDGE_2024:
                return new TunerConstants();
            default:
                // Something went wrong if this branch is reached, by default we will return our Comp Bot
                return new TunerConstantsSmudge();
        }
    }

    static RobotContainerSmudge getRobotContainer(RobotIdentity robot) {
        switch (robot) {
            case SMUDGE_2024:
                return new RobotContainerSmudge();
            case SMIDGE_2024:
                return new RobotContainer();
            default:
                // Something went wrong if this branch is reached, by default we will return our Comp Bot
                return new RobotContainerSmudge();
        }
    }
}
