package com.team1678.frc2024.auto;

import com.team1678.frc2024.auto.routines.AutoRoutine;
import com.team1678.frc2024.auto.routines.DoNothingRoutine;
import com.team1678.frc2024.auto.routines.TestRoutine;
import com.team1678.frc2024.field.AllianceChooser;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoModeSelector {
    private static final String SELECTED_AUTO_MODE = "SELECTED_AUTO_MODE";

    private static final AutoOption DEFAULT_MODE = AutoOption.DO_NOTHING;

    private LoggedDashboardChooser<AutoOption> modeChooser;

    public void initWithDefaults() {
        modeChooser = new LoggedDashboardChooser<>("Auto Routine Chooser");
        modeChooser.addDefaultOption(DEFAULT_MODE.toString(), DEFAULT_MODE);
        modeChooser.addOption(AutoOption.TEST_PATH.toString(), AutoOption.TEST_PATH);
    }

    private AutoRoutine createAutoRoutine(AutoOption option, Boolean isRed) {
        return switch (option) {
            case DO_NOTHING -> new DoNothingRoutine();
            case TEST_PATH -> new TestRoutine();
            default -> {
                System.out.println("ERROR: Unexpected auto routine option: " + option);
                yield new DoNothingRoutine();
            }
        };
    }

    public AutoOption getSelectedAutoEnum() {
        AutoOption option = modeChooser.get();
        if (option == null) {
            return AutoOption.DO_NOTHING;
        }
        return option;
    }

    public AutoRoutine getSelectedAutoRoutine() {
        AutoOption selectedOption = getSelectedAutoEnum();
        return createAutoRoutine(selectedOption, AllianceChooser.getAlliance().equals(DriverStation.Alliance.Red));
    }

    public enum AutoOption {
        DO_NOTHING,
        TEST_PATH;
    }

    public void output() {
        Logger.recordOutput(SELECTED_AUTO_MODE, getSelectedAutoEnum().toString());
    }

}
