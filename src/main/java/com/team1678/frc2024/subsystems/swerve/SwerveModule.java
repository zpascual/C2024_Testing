package com.team1678.frc2024.subsystems.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team1678.frc2024.Ports;
import com.team1678.frc2024.subsystems.ISubsystem;
import com.team1678.lib.drivers.CANDevice;
import com.team1678.lib.drivers.encoders.IEncoder;
import com.team1678.lib.drivers.encoders.MagEncoder;
import com.team1678.lib.drivers.encoders.PheonixProCANCoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;

public abstract class SwerveModule implements ISubsystem {
    protected final IEncoder azmithAbsoluteEncoder;
    protected final int moduleId;
    protected final String moduleName;
    protected final double azmithEncoderOffset;
    private boolean azmithEncoderZeroed = false;
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
    protected abstract StatusCode setAzmithSensorDegrees(double desiredAngle);
    public abstract double getDistanceDrivenMeters();
    public abstract double getDriveVelocityMPS();
    public abstract double getDriveVoltage();
    public abstract boolean isDrivePositionOnTarget();
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDistanceDrivenMeters(), getAngle());
    }
    public abstract void setDriveNeutralMode(NeutralModeValue mode);
    public abstract void setAzmithNeutralMode(NeutralModeValue mode);

    /**
     * @param driveVector A vector whose magnitude represents an open-loop
     * drive speed in the range of [0, 1].
     */
    public abstract void setOpenLoop(Translation2d driveVector);

    /**
     * Causes the module to coast in the specified direction.
     */
    public abstract void setOpenLoopCoast(Rotation2d driveDirection);

    /**
     * @param driveVector A vector whose magnitude represents a drive velocity
     * in inches per second.
     */
    public abstract void setClosedLoopVelocity(Translation2d driveVector);

    /**
     * Causes the module to point in the given direction and maintain a
     * closed-loop velocity of 0.
     */
    public abstract void setClosedLoopVelocityStall(Rotation2d driveDirection);

    /**
     * @param driveVector A vector whose magnitude represents a drive distance
     * in inches.
     */
    public abstract void setClosedLoopPosition(Translation2d driveVector);

    /**
     * Turns off all motors for this module.
     *
     * This is in contrast to the stop() method, which may keep some motors
     * in closed-loop mode to maintain module heading.
     */
    public abstract void disable();

    public void resetAzmithToAbsolute() {
        if (!azmithEncoderZeroed) {
            forceResetAzmithToAbsolute();
        }
    }

    public void forceResetAzmithToAbsolute() {
        if (azmithAbsoluteEncoder.isConnected()) {
            setAzmithSensorDegrees(getAbsoluteEncoderDegrees() - azmithEncoderOffset);
        } else {
            setAzmithSensorDegrees(0.0);
            DriverStation.reportError(String.format("MAG ENCODER FOR %s WAS NOT CONNECTED UPON BOOT!", moduleName), false);
        }

        if (zeroCount < 500) {
            zeroCount++;
        } else {
            System.out.printf("%s ZEROED.%n", moduleName);
            azmithEncoderZeroed = true;
        }
    }

    public void setAzmithMotorZeroed(boolean isZeroed) {
        azmithEncoderZeroed = isZeroed;
    }

    protected String getLogKey(String entryName) {
        return String.format("%s/%s", moduleName, entryName);
    }

    public static class SwerveMotorInfo {
        public final int deviceId;
        public final InvertedValue invertType;
        public final CANDevice.CANBus bus;

        public SwerveMotorInfo(int deviceId, InvertedValue invertType, CANDevice.CANBus bus) {
            this.deviceId = deviceId;
            this.invertType = invertType;
            this.bus = bus;
        }
    }

    public enum EncoderType {
        MAG_ENCODER,
        CANCoder
    }

}
