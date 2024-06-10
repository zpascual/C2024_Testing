package com.team1678.frc2024;

import com.team1678.frc2024.loops.Loop;
import com.team1678.frc2024.subsystems.SubsystemManager;

import java.util.Arrays;
import java.util.List;

public class DriverControls implements Loop {

    private static DriverControls instance = null;
    public static DriverControls getInstance() {
        if (instance == null) {
            instance = new DriverControls();
        }
        return instance;
    }

    private final SubsystemManager subsystemManager;
    public SubsystemManager getSubsystems() {return subsystemManager;}

    public DriverControls() {
        subsystemManager = new SubsystemManager(List.of());
    }

    @Override
    public void onStart(double timestamp) {

    }

    @Override
    public void onLoop(double timestamp) {

    }

    @Override
    public void onStop(double timestamp) {

    }
}
