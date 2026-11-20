package org.firstinspires.ftc.teamcode.Tuning;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "tuning posição")
public class posTuning extends OpMode {

    private Follower follower;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0, 0, Math.toDegrees(0)));
    }

    public Pose getPose() {
        return follower.pose();
    }

    @Override
    public void loop() {
        follower.update();

        telemetry.addData("pos X", getPose().x());
        telemetry.addData("pos Y", getPose().y());
        telemetry.addData("heading", getPose().heading());
        telemetry.update();
    }

}
