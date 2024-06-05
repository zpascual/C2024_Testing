package com.team1678.frc2024.subsystems;

import com.team1678.frc2024.loops.ILooper;
import com.team1678.frc2024.loops.Loop;

import java.util.ArrayList;
import java.util.List;

public class SubsystemManager implements ISubsystem, ILooper, Loop {

    private final List<ISubsystem> mAllSubsystems;
    private final List<Loop> mLoops = new ArrayList<>();

    public SubsystemManager(List<ISubsystem> allSubsystems) {
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
        mAllSubsystems.forEach(ISubsystem::readPeriodicInputs);
    }

    @Override
    public void writePeriodicOutputs() {
        mAllSubsystems.forEach(ISubsystem::writePeriodicOutputs);
    }

    @Override
    public void outputTelemetry() {
        mAllSubsystems.forEach(ISubsystem::outputTelemetry);
    }

    @Override
    public void stop() {
        mAllSubsystems.forEach(ISubsystem::stop);
    }

    @Override
    public void zeroSensors() {
        mAllSubsystems.forEach(ISubsystem::zeroSensors);
    }

    @Override
    public void registerLooper(ILooper looper) {

    }

}
