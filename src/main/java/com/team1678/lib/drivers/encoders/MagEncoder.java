package com.team1678.lib.drivers.encoders;

import com.team1678.frc2024.Settings;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

public class MagEncoder implements IEncoder {
    private final DutyCycle encoder;
    private final double directionSign;

    public MagEncoder(int digitalInputChannel, boolean isReversed) {
        this.encoder = new DutyCycle(new DigitalInput(digitalInputChannel));
        this.directionSign = isReversed ? -1.0 : 1.0;
    }

    public static IEncoder createRealOrSimEncoder(int digitalInputChannel, boolean isReversed) {
        return Settings.kRobotMode == Settings.RobotMode.REAL ? new MagEncoder(digitalInputChannel, isReversed) : new SimulatedAbsoluteEncoder();
    }

    @Override
    public double getDegrees() {
        return directionSign * encoder.getOutput() * 360.0;
    }

    @Override
    public void setPosition(double position) {

    }

    @Override
    public boolean isConnected() {
        return encoder.getFrequency() != 0;
    }
}
