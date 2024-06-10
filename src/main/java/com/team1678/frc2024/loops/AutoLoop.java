package com.team1678.frc2024.loops;

import com.team1678.frc2024.auto.AutoModeSelector;
import com.team1678.frc2024.auto.routines.AutoRoutine;
import com.team1678.frc2024.planners.RequestExecuter;
import org.littletonrobotics.junction.Logger;

public class AutoLoop implements Loop {
    private final AutoModeSelector autoModeSelector;
    private final RequestExecuter requestExecuter = new RequestExecuter();

    public AutoLoop(AutoModeSelector autoModeSelector) {
        this.autoModeSelector = autoModeSelector;
    }

    @Override
    public void onStart(double timestamp) {
        long startTime = Logger.getRealTimestamp();
        AutoRoutine selectedAutoRoutine = autoModeSelector.getSelectedAutoRoutine();
        requestExecuter.request(selectedAutoRoutine.getRoutine());
        long endTime = Logger.getRealTimestamp();
        System.out.printf("Time to construct auto request: %d microseconds%n", endTime - startTime);
    }

    @Override
    public void onLoop(double timestamp) {
        requestExecuter.update();
    }

    @Override
    public void onStop(double timestamp) {
        if (!requestExecuter.isFinished()) {
            System.out.println("Auto mode ended early");
        }
        requestExecuter.clear();
    }
}
