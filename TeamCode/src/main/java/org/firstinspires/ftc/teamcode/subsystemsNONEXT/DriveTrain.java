package org.firstinspires.ftc.teamcode.subsystemsNONEXT;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ConfigRegional.RobotConfig;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class DriveTrain {

    private Follower follower;
    private boolean slowMode = false;

    public void init(HardwareMap hardwareMap) {
        follower = Constants.create(hardwareMap);
        slowMode = false;
    }

    public Follower follower() {
        return follower;
    }

    // Método para ser chamado repetidamente dentro do loop do OpMode (teleOp)
    public void teleopDrive(Gamepad gamepad) {
        drive(
                -gamepad.left_stick_y,
                -gamepad.left_stick_x,
                -gamepad.right_stick_x
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

        // Conversão de campo (Field Centric) baseada no Pedro Pathing
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

        if (AllianceState.current == Alliance.RED) {
            pose = new Pose(
                    RobotConfig.Field.relocRedX,
                    RobotConfig.Field.relocRedY,
                    Math.toRadians(RobotConfig.Field.relocRedHeadingDeg));
        } else {
            pose = new Pose(
                    RobotConfig.Field.relocBlueX,
                    RobotConfig.Field.relocBlueY,
                    Math.toRadians(RobotConfig.Field.relocBlueHeadingDeg));
        }

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

    // Deve ser chamado a cada loop no TeleOp (substitui o periodic() do Mechanism)
    public void update() {
        if (follower != null) {
            follower.update();
        }
    }
}