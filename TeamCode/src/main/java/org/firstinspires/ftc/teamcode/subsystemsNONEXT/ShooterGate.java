package org.firstinspires.ftc.teamcode.subsystemsNONEXT;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ConfigRegional.RobotConfig;

public class ShooterGate {

    private Servo servo;

    private boolean open = false;
    private long lastChangeMs = 0;

    public void init(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "gateShooter");

        servo.setDirection(Servo.Direction.REVERSE);

        close();
    }

    public void open() {
        if (!open) {
            lastChangeMs = System.currentTimeMillis();
        }
        open = true;
    }

    public void close() {
        if (open) {
            lastChangeMs = System.currentTimeMillis();
        }
        open = false;
    }

    public boolean isOpen() {
        return open;
    }

    public boolean isSettled() {
        return (System.currentTimeMillis() - lastChangeMs) >= RobotConfig.Gate.moveTimeMs;
    }

    public void openGate() {
        open();
    }

    public void closeGate() {
        close();
    }

    public void update() {
        if (servo == null) return;

        servo.setPosition(open ? RobotConfig.Gate.openPosition
                : RobotConfig.Gate.closedPosition);
    }
}