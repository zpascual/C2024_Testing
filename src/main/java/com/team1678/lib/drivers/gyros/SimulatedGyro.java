package com.team1678.lib.drivers.gyros;

import edu.wpi.first.math.geometry.Rotation3d;

public class SimulatedGyro implements Gyro {
    @Override
    public void setYaw(double angle) {

    }

    @Override
    public double getYaw() {
        return 0;
    }

    @Override
    public double getPitch() {
        return 0;
    }

    @Override
    public double getRoll() {
        return 0;
    }

    @Override
    public Rotation3d getYPR() {
        return new Rotation3d(0,0,0);
    }

    @Override
    public void reset() {

    }
}
