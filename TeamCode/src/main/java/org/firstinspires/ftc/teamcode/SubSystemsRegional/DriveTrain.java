package org.firstinspires.ftc.teamcode.SubSystemsRegional;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ConfigRegional.RobotConfig;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import dev.easyfamily.EasyStack.gamepad.EasyGamepad;
import dev.easyfamily.EasyStack.subsytem.EasySubsystemBase;

public class DriveTrain extends EasySubsystemBase {


    private Follower follower;
    private boolean slowMode = false;

    @Override
    public void init(HardwareMap hardwareMap) {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(56, 8, 0));
        slowMode = false;
    }

    @Override
    public void loop() {
        if (follower != null) {
            follower.update();
        }
    }

    public Follower follower() {
        return follower;
    }

    public void teleopDrive(EasyGamepad driver) {
        drive(
                -driver.leftStickY(),
                -driver.leftStickX(),
                -driver.rightStickX()
        );
    }

    public void setSlowMode(boolean on) {
        slowMode = on;
    }

    private void drive(double forwardInput, double lateralInput, double turnInput) {
        double forward = shape(forwardInput);
        double lateral = shape(lateralInput);
        double turn = shape(turnInput) * RobotConfig.Drive.turnSensitivity;

        double scale = slowMode ? RobotConfig.Drive.slowModeFactor : 1.0;

        double heading = getDriverHeading();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double robotForward = forward * cos + lateral * sin;
        double robotLateral = -forward * sin + lateral * cos;

        follower.manual(
                robotForward * scale,
                robotLateral * scale,
                turn * scale
        );
    }

    public void resetHeading() {
        Pose currentPose = follower.pose();

        follower.setPose(new Pose(
                currentPose.x(),
                currentPose.y(),
                0
        ));
    }

    public void relocalize() {
        Pose pose;

        pose = new Pose(
                RobotConfig.Field.relocX,
                RobotConfig.Field.relocY,
                RobotConfig.Field.relocHeading
        );

        follower.setPose(pose);
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public Pose getPose() {
        return follower.pose();
    }

    public double getHeading() {
        return follower.pose().heading();
    }

    private double getDriverHeading() {
        return getHeading();
    }

    private double shape(double input) {
        double deadzone = RobotConfig.Drive.stickDeadzone;
        double magnitude = Math.abs(input);

        if (magnitude < deadzone) {
            return 0;
        }

        magnitude = (magnitude - deadzone) / (1.0 - deadzone);

        return Math.copySign(
                Math.pow(magnitude, RobotConfig.Drive.stickExponent),
                input);
    }
}