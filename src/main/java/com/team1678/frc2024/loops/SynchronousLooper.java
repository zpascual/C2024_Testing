package com.team1678.frc2024.loops;

import com.team1678.frc2024.subsystems.SubsystemManager;

import java.util.ArrayList;
import java.util.List;

public class SynchronousLooper implements ILooper {
    private final SubsystemManager subsystemManager;
    private final List<Loop> autoLoops = new ArrayList<>();
    private final List<Loop> teleopLoops = new ArrayList<>();
    private final List<Loop> disabledLoops = new ArrayList<>();
    private EnabledMode lastEnabledMode = EnabledMode.TELEOP;

    public SynchronousLooper(SubsystemManager subsystemManager) {
        this.subsystemManager = subsystemManager;
    }

    public void registerAutoLoop(Loop loop) {
        autoLoops.add(loop);
    }

    public void registerTeleopLoop(Loop loop) {
        teleopLoops.add(loop);
    }

    public void registerDisabledLoop(Loop loop) {
        disabledLoops.add(loop);
    }

    @Override
    public void register(Loop loop) {
        registerAutoLoop(loop);
        registerTeleopLoop(loop);
        registerDisabledLoop(loop);
    }

    private void startEnabled(double timestamp, List<Loop> auxLoops) {
        disabledLoops.forEach(l -> l.onStop(timestamp));
        subsystemManager.onStart(timestamp);
        auxLoops.forEach(l -> l.onStart(timestamp));
    }

    public void startAuto(double timestamp) {
        startEnabled(timestamp, autoLoops);
        lastEnabledMode = EnabledMode.AUTO;
    }

    public void startTeleop(double timestamp) {
        startEnabled(timestamp, teleopLoops);
        lastEnabledMode = EnabledMode.TELEOP;
    }

    public void startDisabled(double timestamp) {
        switch (lastEnabledMode) {
            case AUTO -> autoLoops.forEach(l -> l.onStop(timestamp));
            case TELEOP -> teleopLoops.forEach(l -> l.onStop(timestamp));
        }
        subsystemManager.onStop(timestamp);
        subsystemManager.stop();
        disabledLoops.forEach(l -> l.onStart(timestamp));
    }

    private void onEnabledLoop(double timestamp, List <Loop> auxLoops) {
        subsystemManager.readPeriodicInputs();
        auxLoops.forEach(l -> l.onLoop(timestamp));
        subsystemManager.onLoop(timestamp);
        subsystemManager.writePeriodicOutputs();
        subsystemManager.outputTelemetry();
    }

    public void onAutoLoop(double timestamp) {
        onEnabledLoop(timestamp, autoLoops);
    }

    public void onTeleopLoop(double timestamp) {
        onEnabledLoop(timestamp, teleopLoops);
    }

    public void onDisabledLoop(double timestamp) {
        subsystemManager.readPeriodicInputs();
        disabledLoops.forEach(l -> l.onLoop(timestamp));
        subsystemManager.writePeriodicOutputs();
        subsystemManager.outputTelemetry();
    }

    private enum EnabledMode {
        AUTO, TELEOP
    }

}
