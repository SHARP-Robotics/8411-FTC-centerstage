package org.firstinspires.ftc.teamcode.util.fake;

import com.qualcomm.robotcore.hardware.HardwareDevice;

public class HardwareDeviceFake implements HardwareDevice {
    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return null;
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}