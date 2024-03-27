package org.firstinspires.ftc.teamcode.util.fake;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoController;

public class CRServoFake extends HardwareDeviceFake implements CRServo {
    @Override
    public ServoController getController() {
        return null;
    }

    @Override
    public int getPortNumber() {
        return 0;
    }

    @Override
    public void setDirection(Direction direction) {

    }

    @Override
    public Direction getDirection() {
        return null;
    }

    @Override
    public void setPower(double power) {

    }

    @Override
    public double getPower() {
        return 0;
    }
}