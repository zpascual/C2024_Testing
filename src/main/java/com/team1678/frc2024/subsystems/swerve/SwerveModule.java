package com.team1678.frc2024.subsystems.swerve;

import com.team1678.frc2024.Constants;
import com.team1678.frc2024.Ports;
import com.team1678.frc2024.loops.ILooper;
import com.team1678.frc2024.subsystems.ISubsystem;
import com.team1678.lib.sensors.encoders.IEncoder;
import com.team1678.lib.sensors.encoders.MagEncoder;
import com.team1678.lib.sensors.encoders.PheonixProCANCoder;
import com.team254.lib.geometry.Rotation2d;

public abstract class SwerveModule implements ISubsystem {
    protected final IEncoder azmithAbsoluteEncoder;
    protected final int moduleId;
    protected final String moduleName;
    protected final double azmithEncoderOffset;
    private int zeroCount = 0;

    public SwerveModule(int moduleId, double azmithEncoderOffset, boolean flipAzmithEncoder, EncoderType azmithEncoderType) {
        this.moduleId  = moduleId;
        this.moduleName = String.format("Module %s", moduleId);

        this.azmithAbsoluteEncoder = azmithEncoderType == EncoderType.MAG_ENCODER ? new MagEncoder(Ports.kModuleEncoders[moduleId].getDeviceNumber(), flipAzmithEncoder) : new PheonixProCANCoder(Ports.kModuleEncoders[moduleId].getDeviceNumber(), flipAzmithEncoder, azmithEncoderOffset);
        this.azmithEncoderOffset = azmithEncoderOffset;
    }

    public abstract double getAbsoluteEncoderDegrees();
    public abstract Rotation2d getAngle();
    public abstract boolean isAngleOnTarget();
    protected abstract ErrorCode setAzmithSensorDegrees(double desiredAngle);
    public abstract double getDistanceDriven();
    public abstract double getDriveVelocity();
    public abstract double getDriveVoltage();
    public abstract boolean isDrivePositionOnTarget();

    @Override
    public void readPeriodicInputs() {

    }

    @Override
    public void writePeriodicOutputs() {

    }

    @Override
    public void outputTelemetry() {

    }

    @Override
    public void stop() {

    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void registerLooper(ILooper looper) {

    }

    public enum EncoderType {
        MAG_ENCODER,
        CANCoder
    }

}
