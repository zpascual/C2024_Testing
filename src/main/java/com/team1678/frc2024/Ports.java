package com.team1678.frc2024;

import com.team1678.lib.drivers.CANDevice;
import com.team1678.lib.drivers.DIODevice;

public class Ports {
    // Motors
    public static final CANDevice FRONT_RIGHT_ROTATION = new CANDevice(0, CANDevice.CANBus.MAIN);
    public static final CANDevice FRONT_RIGHT_DRIVE = new CANDevice(1, CANDevice.CANBus.MAIN);
    public static final CANDevice FRONT_LEFT_ROTATION = new CANDevice(2, CANDevice.CANBus.MAIN);
    public static final CANDevice FRONT_LEFT_DRIVE = new CANDevice(3, CANDevice.CANBus.MAIN);
    public static final CANDevice REAR_LEFT_ROTATION = new CANDevice(4, CANDevice.CANBus.MAIN);
    public static final CANDevice REAR_LEFT_DRIVE = new CANDevice(5, CANDevice.CANBus.MAIN);
    public static final CANDevice REAR_RIGHT_ROTATION = new CANDevice(6, CANDevice.CANBus.MAIN);
    public static final CANDevice REAR_RIGHT_DRIVE = new CANDevice(7, CANDevice.CANBus.MAIN);
    public static final CANDevice INTAKE = new CANDevice(8, CANDevice.CANBus.SECONDARY);
    public static final CANDevice INTAKE_WRIST = new CANDevice(9, CANDevice.CANBus.MAIN);
    public static final CANDevice SERIALIZER = new CANDevice(10, CANDevice.CANBus.MAIN);
    public static final CANDevice FEEDER = new CANDevice(11, CANDevice.CANBus.MAIN);
    public static final CANDevice AMP_ROLLER = new CANDevice(12, CANDevice.CANBus.SECONDARY);
    public static final CANDevice ELEVATOR_LEADER = new CANDevice(13, CANDevice.CANBus.MAIN);
    public static final CANDevice ELEVATOR_FOLLOWER = new CANDevice(14, CANDevice.CANBus.MAIN);
    public static final CANDevice SHOOTER_TOP = new CANDevice(15, CANDevice.CANBus.MAIN);
    public static final CANDevice SHOOTER_BOTTOM = new CANDevice(16, CANDevice.CANBus.MAIN);
    public static final CANDevice HOOD = new CANDevice(17, CANDevice.CANBus.MAIN);
    public static final CANDevice CLIMBER_LEADER = new CANDevice(18, CANDevice.CANBus.MAIN);
    public static final CANDevice CLIMBER_FOLLOWER = new CANDevice(19, CANDevice.CANBus.MAIN);

    // Sensors
    public static final CANDevice PIGEON = new CANDevice(30, CANDevice.CANBus.MAIN);
    public static final CANDevice FRONT_RIGHT_CANCODER = new CANDevice(31, CANDevice.CANBus.MAIN);
    public static final CANDevice FRONT_LEFT_CANCODER = new CANDevice(32, CANDevice.CANBus.MAIN);
    public static final CANDevice REAR_LEFT_CANCODER = new CANDevice(33, CANDevice.CANBus.MAIN);
    public static final CANDevice REAR_RIGHT_CANCODER = new CANDevice(34, CANDevice.CANBus.MAIN);

    //MISC
    public static final CANDevice CAN__DLE = new CANDevice(40, CANDevice.CANBus.SECONDARY);

    // DIO
    public static final DIODevice SERIALIZER_BEAM_BREAK = new DIODevice(0);
    public static final DIODevice FEEDER_BEAM_BREAK = new DIODevice(1);
    public static final DIODevice AMP_BEAM_BREAK = new DIODevice(2);

    // Groupings
    public static final CANDevice[] kModuleEncoders = new CANDevice[] {
      FRONT_RIGHT_CANCODER, FRONT_LEFT_CANCODER, REAR_LEFT_CANCODER, REAR_RIGHT_CANCODER
    };
}
