package org.firstinspires.ftc.teamcode.subsystemsNONEXT;

import static org.firstinspires.ftc.teamcode.ConfigRegional.RobotConfig.FlowerRemover.upPosition;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ConfigRegional.RobotConfig;

public class FlowerRemover {

    private Servo servoLeft;
    private Servo servoRight;

    private boolean down = false;
    private long lastChangeMs = 0;

    private static final double RIGHT_OFFSET = 0.1;

    public void init(HardwareMap hardwareMap) {
        servoLeft = hardwareMap.get(Servo.class, "flowerServoLeft");
        servoRight = hardwareMap.get(Servo.class, "flowerServoRight");

        initialize();
    }

    public void down() {
        if (!down) {
            lastChangeMs = System.currentTimeMillis();
        }
        down = true;
    }

    public void up() {
        if (down) {
            lastChangeMs = System.currentTimeMillis();
        }
        down = false;
    }

    public void initialize() {
        down = false;
        lastChangeMs = System.currentTimeMillis();
        if (servoLeft != null && servoRight != null) {
            servoLeft.setPosition(upPosition);
            servoRight.setPosition(1.0 - upPosition + RIGHT_OFFSET);
        }
    }

    public boolean isDown() {
        return down;
    }

    public boolean isSettled() {
        return (System.currentTimeMillis() - lastChangeMs) >= RobotConfig.FlowerRemover.moveTimeMs;
    }

    public void setDown() {
        down();
    }

    public void setUp() {
        up();
    }

    public void setInit() {
        initialize();
    }

    public void update() {
        if (servoLeft == null || servoRight == null) return;

        double targetPosition = down ? RobotConfig.FlowerRemover.downPosition
                : upPosition;

        servoRight.setPosition(1.0 - targetPosition + RIGHT_OFFSET);
        servoLeft.setPosition(targetPosition);
    }
}