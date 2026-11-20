package org.firstinspires.ftc.teamcode.SubSystemsRegional;

import static org.firstinspires.ftc.teamcode.ConfigRegional.RobotConfig.FlowerRemover.downPositionRight;
import static org.firstinspires.ftc.teamcode.ConfigRegional.RobotConfig.FlowerRemover.upPositionRight;

import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.easyfamily.EasyStack.hardware.Servo.EasyServo;
import dev.easyfamily.EasyStack.hardware.Servo.EasyServoGroup;
import dev.easyfamily.EasyStack.subsytem.EasySubsystemBase;

public class FlowerRemover extends EasySubsystemBase {

    private EasyServo rightFlower, leftFlower;

    @Override
    public void init(HardwareMap hardwareMap) {
        rightFlower = new EasyServo(hardwareMap, "rightFlower");
        rightFlower.addState("Up", upPositionRight);
        rightFlower.addState("Down", downPositionRight);;

        rightFlower.setState("Up");
    }

    public void up() {
        rightFlower.setState("Up");
    }

    public void down() {
        rightFlower.setState("Down");
    }

}
