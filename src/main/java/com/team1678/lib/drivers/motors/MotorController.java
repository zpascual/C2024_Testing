package com.team1678.lib.drivers.motors;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public interface MotorController {
    public void configureAsRoller();
    public void configureAsServo();
    public void configureAsCoaxialSwerveRotation();
    public void configureAsCoaxialSwerveDrive();
    public void configureAsDifferentialSwerveMotor();

    public StatusCode configForwardSoftLimitThreshold(double encoderUnits);
    public StatusCode configReverseSoftLimitThreshold(double encoderUnits);
    public StatusCode configForwardSoftLimitEnable(boolean enable);
    public StatusCode configReverseSoftLimitEnable(boolean enable);

    public StatusCode configMotionCruiseVelocity(double encoderUnitsPer100Ms);
    public StatusCode configMotionAcceleration(double encoderUnitsPer100MsPerSecond);

    public StatusCode configSupplyCurrentLimit(double amps, double timeThreshold);
    public StatusCode configCurrentLimits(CurrentLimitsConfigs currentLimitConfig);
    public StatusCode configStatorCurrentLimit(double amps);
    public StatusCode disableStatorCurrentLimit();
    public StatusCode disableSupplyCurrentLimit();

    public double getSupplyAmps();
    public double getStatorAmps();
    public double getAppliedVoltage();
    public double getMotorTemperature();

    public double getVelocityEncoderUnitsPer100Ms();
    public double getSelectedSensorPosition();
    public double getInternalRotorVelocity();
    public double getInternalRotorPosition();
    public StatusCode setSelectedSensorPosition(double encoderUnits);

    public void useIntegratedSensor();
    public void useCANCoder(int cancoderId);

    public StatusCode setMotorOutputConfig(MotorOutputConfigs motorOutputConfig);
    public StatusCode setNeutralModeValue(NeutralModeValue neutralMode);
    public StatusCode setInverted(InvertedValue invertedValue);

    public void setPIDF(MotorPIDF pidf);
    public void selectProfileSlot(int slotIndex);

    public void set(ControlModeValue mode, double demand);
    public void set(ControlModeValue mode, double demand, double arbitraryFeedForward);

    public boolean isConnected();

    public static class MotorPIDF {
        public final int slotIndex;
        public final double kP;
        public final double kI;
        public final double kD;
        public final double kF;

        public MotorPIDF(int slotIndex, double kP, double kI, double kD, double kF) {
            this.slotIndex = slotIndex;
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
        }
    }
}
