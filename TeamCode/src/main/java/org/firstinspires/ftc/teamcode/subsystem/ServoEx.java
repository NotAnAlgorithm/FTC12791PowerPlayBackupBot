package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ServoEx implements Servo {
    private final Servo servo;
    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double startPosition;
    private double targetPosition;
    private double targetTime;
    private boolean busy = false;

    public ServoEx(Servo servo) {
        this.servo = servo;
    }

    @Override
    public ServoController getController() {
        return servo.getController();
    }

    @Override
    public int getPortNumber() {
        return servo.getPortNumber();
    }

    @Override
    public void setDirection(Direction direction) {
        servo.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return servo.getDirection();
    }

    @Override
    public void setPosition(double position) {
        servo.setPosition(position);
    }

    public void setPosition(double position, double time) {
        startPosition = getPosition();
        targetPosition = position;
        targetTime = time;
        busy = true;
        timer.reset();
    }

    @Override
    public double getPosition() {
        return servo.getPosition();
    }

    @Override
    public void scaleRange(double min, double max) {
        servo.scaleRange(min, max);
    }

    @Override
    public Manufacturer getManufacturer() {
        return servo.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return servo.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return servo.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return servo.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        servo.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        servo.close();
    }

    public boolean isBusy() {
        return busy;
    }

    public void update() {
        if (busy) {
            setPosition((targetPosition - startPosition) * timer.time() / targetTime + startPosition);
            if (timer.time() > targetTime) {
                setPosition(targetPosition);
                busy = false;
            }
        }
    }
}
