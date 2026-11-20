package org.firstinspires.ftc.teamcode.SubSystemsRegional;

import static org.firstinspires.ftc.teamcode.ConfigRegional.RobotConfig.Gate.moveTimeMS;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.ConfigRegional.RobotConfig;
import dev.easyfamily.EasyStack.hardware.Servo.EasyServo;
import dev.easyfamily.EasyStack.subsytem.EasySubsystemBase;

public class Gate extends EasySubsystemBase {

    private EasyServo servo;
    private boolean open = false;
    private long openTimestamp = 0;
    private boolean timerStarted = false;

    @Override
    public void init(HardwareMap hardwareMap) {
        servo = new EasyServo(hardwareMap, "gateShooter");
        servo.addState("CLOSED", RobotConfig.Gate.closedPosition);
        servo.addState("OPEN", RobotConfig.Gate.openPosition);
        close();
    }

    public void open() {
        if (!open && servo != null) {
            open = true;
            servo.setState("OPEN");
            openTimestamp = System.currentTimeMillis();
            timerStarted = true;
        }
    }

    public void close() {
        open = false;
        if (servo != null) {
            servo.setState("CLOSED");
        }
        timerStarted = false;
    }

    public boolean isOpen() {
        return open;
    }
    public boolean hasWaitTimeElapsed() {
        if (!timerStarted) return false;
        return (System.currentTimeMillis() - openTimestamp) >= moveTimeMS;
    }
}