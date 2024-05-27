package com.team1678.lib.drivers.motors;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class SimulatedMotorController implements MotorController {

    @Override
    public void configureAsRoller() {

    }

    @Override
    public void configureAsServo() {

    }

    @Override
    public void configureAsCoaxialSwerveRotation() {

    }

    @Override
    public void configureAsCoaxialSwerveDrive() {

    }

    @Override
    public void configureAsDifferentialSwerveMotor() {

    }

    @Override
    public StatusCode configForwardSoftLimitThreshold(double encoderUnits) {
        return StatusCode.OK;
    }

    @Override
    public StatusCode configReverseSoftLimitThreshold(double encoderUnits) {
        return StatusCode.OK;
    }

    @Override
    public StatusCode configForwardSoftLimitEnable(boolean enable) {
        return StatusCode.OK;
    }

    @Override
    public StatusCode configReverseSoftLimitEnable(boolean enable) {
        return StatusCode.OK;
    }

    @Override
    public StatusCode configMotionCruiseVelocity(double encoderUnitsPer100Ms) {
        return StatusCode.OK;
    }

    @Override
    public StatusCode configMotionAcceleration(double encoderUnitsPer100MsPerSecond) {
        return StatusCode.OK;
    }

    @Override
    public StatusCode configSupplyCurrentLimit(double amps, double timeThreshold) {
        return StatusCode.OK;
    }

    @Override
    public StatusCode configCurrentLimits(CurrentLimitsConfigs currentLimitConfig) {
        return StatusCode.OK;
    }

    @Override
    public StatusCode configStatorCurrentLimit(double amps) {
        return StatusCode.OK;
    }

    @Override
    public StatusCode disableStatorCurrentLimit() {
        return StatusCode.OK;
    }

    @Override
    public StatusCode disableSupplyCurrentLimit() {
        return StatusCode.OK;
    }

    @Override
    public double getSupplyAmps() {
        return 0;
    }

    @Override
    public double getStatorAmps() {
        return 0;
    }

    @Override
    public double getAppliedVoltage() {
        return 0;
    }

    @Override
    public double getMotorTemperature() {
        return 0;
    }

    @Override
    public double getVelocityEncoderUnitsPer100Ms() {
        return 0;
    }

    @Override
    public double getSelectedSensorPosition() {
        return 0;
    }

    @Override
    public double getInternalRotorVelocity() {
        return 0;
    }

    @Override
    public double getInternalRotorPosition() {
        return 0;
    }

    @Override
    public StatusCode setSelectedSensorPosition(double encoderUnits) {
        return StatusCode.OK;
    }

    @Override
    public void useIntegratedSensor() {

    }

    @Override
    public void useCANCoder(int cancoderId) {

    }

    @Override
    public StatusCode setMotorOutputConfig(MotorOutputConfigs motorOutputConfig) {
        return StatusCode.OK;
    }

    @Override
    public StatusCode setNeutralModeValue(NeutralModeValue neutralMode) {
        return StatusCode.OK;
    }

    @Override
    public StatusCode setInverted(InvertedValue invertedValue) {
        return StatusCode.OK;
    }

    @Override
    public void setPIDF(MotorPIDF pidf) {

    }

    @Override
    public void selectProfileSlot(int slotIndex) {

    }

    @Override
    public void set(ControlModeValue mode, double demand) {

    }

    @Override
    public void set(ControlModeValue mode, double demand, double arbitraryFeedForward) {

    }

    @Override
    public boolean isConnected() {
        return true;
    }
}
