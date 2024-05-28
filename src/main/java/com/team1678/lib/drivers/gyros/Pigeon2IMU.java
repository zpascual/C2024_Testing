package com.team1678.lib.drivers.gyros;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.team1678.frc2024.Constants;
import com.team1678.frc2024.Robot;
import com.team1678.lib.drivers.CANDevice;
import edu.wpi.first.math.geometry.Rotation3d;

public class Pigeon2IMU implements Gyro {
    Pigeon2 pigeon;

    private double rollOffset = 0.0;

    public static Gyro createRealorSimulatedGyro(CANDevice device) {
        return Robot.isReal() ? new Pigeon2IMU(device) : new SimulatedGyro();
    }

    private Pigeon2IMU(CANDevice device) {
        pigeon = new Pigeon2(device.getDeviceNumber(), device.getBusName());
    }

    @Override
    public void setYaw(double angle) {
        pigeon.setYaw(angle, Constants.CANTiming.kCANTimeoutMs);
    }

    @Override
    public double getYaw() {
        return pigeon.getYaw().getValue();
    }

    @Override
    public double getPitch() {
        return pigeon.getPitch().getValue();
    }

    @Override
    public double getRoll() {
        return pigeon.getRoll().getValue();
    }

    @Override
    public Rotation3d getYPR() {
        return pigeon.getRotation3d();
    }

    @Override
    public void reset() {
        pigeon.reset();
    }
}
