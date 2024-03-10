package frc.robot.config;

import static frc.robot.util.MacAddress.*;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.MacAddress;

public enum RobotIdentity {
    SMUDGE_2024,
    SMIDGE_2024,
    SIMULATION;

    public static RobotIdentity getIdentity() {
        if (Robot.isReal()) {
            String mac = getMACAddress();
            if (!mac.equals("")) {
                if (mac.equals(MacAddress.SMIDGE_MAC)) {
                    return SMIDGE_2024;
                } else if (mac.equals(MacAddress.SMUDGE_MAC)) {
                    return SMUDGE_2024;
                }
            }
            if (Constants.compRobot == SMUDGE_2024) {
                return SMUDGE_2024;
            } else {
                return SMIDGE_2024;
            }
        } else {
            return SIMULATION;
        }
    }
}
/*public enum RobotIdentity {
   COMPETITION_ROBOT_2024,
   PRACTICE_ROBOT_2024,
   SIMULATION,

   public static RobotIdentity getIdentity() {
       if (Robot.isReal()) {
           String mac = getMACAddress();
           if (!mac.equals("")) {
               if (mac.equals(MacAddress.ROBOT_ONE_MAC) || mac.equals(SECONDARY_ROBOT_ONE_MAC)) {
                   return COMPETITION_ROBOT_2024;
               } else {
                   return PRACTICE_ROBOT_2024;
               }

       } else {
           return SIMULATION;
       }
   }
}*/
