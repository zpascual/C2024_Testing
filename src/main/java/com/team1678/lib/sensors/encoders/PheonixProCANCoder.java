package com.team1678.lib.sensors.encoders;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.team1678.frc2024.Robot;
import com.team1678.frc2024.Settings;
import com.team1678.lib.drivers.CANDevice;

public class PheonixProCANCoder implements IEncoder {
    private CANcoder encoder = null;

    public PheonixProCANCoder(int deviceId, String bus, boolean isReversed, double magnetAngleOffset) {
        this.encoder = new CANcoder(deviceId, bus);
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.withSensorDirection(isReversed ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive);
        config.MagnetSensor.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf);
        config.MagnetSensor.withMagnetOffset(magnetAngleOffset);
        encoder.getConfigurator().apply(config);
    }

    public PheonixProCANCoder(int deviceId, boolean isReversed, double magnetAngleOffset) {
        new PheonixProCANCoder(deviceId, CANDevice.CANBus.MAIN.toString(), isReversed, magnetAngleOffset);
    }

    public static IEncoder createRealOrSimEncoder(int deviceId, String bus, boolean isReversed, double magnetAngleOffset) {
        return Settings.kRobotMode == Settings.RobotMode.REAL ? new PheonixProCANCoder(deviceId, bus, isReversed, magnetAngleOffset) : new SimulatedAbsoluteEncoder();
    }

    public static IEncoder createRealOrSimEncoder(int deviceId, boolean isReversed, double magnetAngleOffset) {
        return createRealOrSimEncoder(deviceId, CANDevice.CANBus.MAIN.toString(), isReversed, magnetAngleOffset);
    }

    @Override
    public double getDegrees() {
        return encoder.getAbsolutePosition().getValue() * 360.0;
    }

    @Override
    public void setPosition(double position) {
        encoder.setPosition(position / 360.0);
    }

    @Override
    public boolean isConnected() {
        return encoder.getAbsolutePosition().getStatus().isOK();
    }
}
