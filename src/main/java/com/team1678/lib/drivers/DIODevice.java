package com.team1678.lib.drivers;

import com.team1678.frc2024.Settings;

public class DIODevice {
    private final int mDeviceNumber;
    private final int mSecondaryDeviceNumber;

    public DIODevice(int mDeviceNumber) {
        this.mDeviceNumber = mDeviceNumber;
        this.mSecondaryDeviceNumber = 9999;
    }

    public DIODevice(int mDeviceNumber, int mSecondaryDeviceNumber) {
        this.mDeviceNumber = mDeviceNumber;
        this.mSecondaryDeviceNumber = mSecondaryDeviceNumber;
    }

    public int getDeviceNumber() {
        if (Settings.kIsUsingCompBot || mSecondaryDeviceNumber == 9999) {
            return mDeviceNumber;
        }
        return mSecondaryDeviceNumber;
    }

}
