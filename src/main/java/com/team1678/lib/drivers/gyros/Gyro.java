package com.team1678.lib.drivers.gyros;

import edu.wpi.first.math.geometry.Rotation3d;

public interface Gyro {
    public void setYaw(double angle);
    public double getYaw();
    public double getPitch();
    public double getRoll();
    public Rotation3d getYPR();
    public void reset();
}
