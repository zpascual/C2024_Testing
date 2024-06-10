package com.team1678.frc2024.field;

import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Single source of truth for what alliance the robot is actually on
 */
public class AllianceChooser {
    private static final AllianceChooserInputs inputs = new AllianceChooserInputs();

    public static DriverStation.Alliance getAlliance() {return inputs.alliance;}

    public static void update() {
        inputs.alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
        Logger.processInputs("Alliance Chooser", inputs);
    }

    private static class AllianceChooserInputs implements LoggableInputs {
        DriverStation.Alliance alliance = DriverStation.Alliance.Red;

        @Override
        public void toLog(LogTable table) {
            table.put("Alliance", alliance.toString());
        }

        @Override
        public void fromLog(LogTable table) {
            alliance = table.get("Alliance", alliance.toString() == "Blue" ? DriverStation.Alliance.Blue : DriverStation.Alliance.Red);
        }
    }

}
