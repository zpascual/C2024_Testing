package frc.robot.team1678.lib.drivers;

import frc.robot.team1678.frc2024.Settings;

public class CANDevice {

    private final int mDeviceNumber;
    private final int mSecondaryDeviceNumber;
    private final CANBus mBusName;

    /**
     * Constructor to simplify CAN device port mapping
     * @param mDeviceNumber CAN ID given in TunerX
     * @param mBusName  CAN bus type
     */
    public CANDevice(int mDeviceNumber, CANBus mBusName) {
        this.mDeviceNumber = mDeviceNumber;
        this.mSecondaryDeviceNumber = 9999;
        this.mBusName = mBusName;
    }

    /**
     * Constructor to simplify CAN device port mapping with a secondary bot
     * @param mDeviceNumber CAN ID given for primary bot given in TunerX
     * @param mSecondaryDeviceNumber CAN ID given for secondary bot given in TunerX
     * @param mBusName CAN bus type
     */
    public CANDevice(int mDeviceNumber, int mSecondaryDeviceNumber, CANBus mBusName) {
        this.mDeviceNumber = mDeviceNumber;
        this.mSecondaryDeviceNumber = mSecondaryDeviceNumber;
        this.mBusName = mBusName;
    }

    public int getDeviceNumber() {
        if (Settings.kIsUsingCompBot || mSecondaryDeviceNumber == 9999) {
            return mDeviceNumber;
        }
        return mSecondaryDeviceNumber;
    }

    public String getBusName() {return mBusName.name;}

    public enum CANBus {
        MAIN("canivore"),
        SECONDARY("rio");

        final String name;
        CANBus(String name) {
            this.name = name;
        }
    }

}
