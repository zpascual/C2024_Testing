package com.team1678.lib.drivers.encoders;

public class SimulatedAbsoluteEncoder implements IEncoder {
    @Override
    public double getDegrees() {
        return 0.0;
    }

    @Override
    public void setPosition(double position) {

    }

    @Override
    public boolean isConnected() {
        return true;
    }
}
