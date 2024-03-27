package org.firstinspires.ftc.teamcode.util.fake;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class ServoFake extends HardwareDeviceFake implements Servo, PwmControl {
    private double position = 0;
    private Direction direction = Direction.FORWARD;
    private PwmRange pwmRange = new PwmRange(600, 2400);
    private boolean isPwmEnabled = true;

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
        this.direction = direction;
    }

    @Override
    public Direction getDirection() {
        return direction;
    }

    @Override
    public void setPosition(double position) {
        this.position = position;
    }

    @Override
    public double getPosition() {
        return position;
    }

    @Override
    public void scaleRange(double min, double max) {

    }

    @Override
    public void setPwmRange(PwmRange range) {
        pwmRange = range;
    }

    @Override
    public PwmRange getPwmRange() {
        return pwmRange;
    }

    @Override
    public void setPwmEnable() {
        isPwmEnabled = true;
    }

    @Override
    public void setPwmDisable() {
        isPwmEnabled = false;
    }

    @Override
    public boolean isPwmEnabled() {
        return isPwmEnabled;
    }
}