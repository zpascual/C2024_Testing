package com.team1678.frc2024.subsystems;

import com.team1678.frc2024.loops.ILooper;
import com.team1678.frc2024.loops.Loop;

import java.util.ArrayList;
import java.util.List;

public class SubsystemManager extends Subsystem implements ILooper, Loop {

    private final List<Subsystem> mAllSubsystems;
    private final List<Loop> mLoops = new ArrayList<>();

    public SubsystemManager(List<Subsystem> allSubsystems) {
        this.mAllSubsystems = allSubsystems;
        allSubsystems.forEach(s -> s.registerLooper(this));
    }

    @Override
    public void register(Loop loop) {
        mLoops.add(loop);
    }

    @Override
    public void onStart(double timestamp) {
        mLoops.forEach(l -> l.onStart(timestamp));
    }

    @Override
    public void onLoop(double timestamp) {
        mLoops.forEach(l -> l.onLoop(timestamp));
    }

    @Override
    public void onStop(double timestamp) {
        mLoops.forEach(l -> l.onStop(timestamp));
    }

    @Override
    public void readPeriodicInputs() {
        mAllSubsystems.forEach(Subsystem::readPeriodicInputs);
    }

    @Override
    public void writePeriodicOutputs() {
        mAllSubsystems.forEach(Subsystem::writePeriodicOutputs);
    }

    @Override
    public void outputTelemetry() {
        mAllSubsystems.forEach(Subsystem::outputTelemetry);
    }

    @Override
    public void stop() {
        mAllSubsystems.forEach(Subsystem::stop);
    }

    @Override
    public void zeroSensors() {
        mAllSubsystems.forEach(Subsystem::zeroSensors);
    }

    @Override
    public void registerLooper(ILooper looper) {

    }

}
