package com.team1678.lib.sensors.encoders;

public interface IEncoder {
    double getDegrees();

    void setPosition(double position);

    boolean isConnected();
}
