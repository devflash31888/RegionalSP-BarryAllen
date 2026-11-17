package org.firstinspires.ftc.teamcode.SubSystemsRegional;

import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.easyfamily.EasyStack.hardware.Servo.EasyServo;
import dev.easyfamily.EasyStack.hardware.Servo.EasyServoGroup;
import dev.easyfamily.EasyStack.subsytem.EasySubsystemBase;

public class FlowerRemover extends EasySubsystemBase {

    private EasyServo rightFlower, leftFlower;
    private EasyServoGroup flower;

    @Override
    public void init(HardwareMap hardwareMap) {
        rightFlower = new EasyServo(hardwareMap, "rightFlower");
        leftFlower = new EasyServo(hardwareMap, "leftFlower").setInverted(true);

        flower = new EasyServoGroup(rightFlower, leftFlower)
                .setLimits(0.2, 0.55)
                .addState("Up", 0.2)
                .addState("Down", 0.55);

        flower.setState("Up");
    }

    public void up() {
        flower.setState("Up");
    }

    public void down() {
        flower.setState("Down");
    }

}
