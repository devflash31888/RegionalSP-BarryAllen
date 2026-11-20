package org.firstinspires.ftc.teamcode.OpModesRegional.auto;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.SubSystemsRegional.FlowerRemover;
import org.firstinspires.ftc.teamcode.SubSystemsRegional.Gate;
import org.firstinspires.ftc.teamcode.SubSystemsRegional.Intake;
import org.firstinspires.ftc.teamcode.localization.PoseManager;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Auto Maior")
public class AutoInicial extends OpMode {

    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Intake intake = new Intake();
    private final Gate gate = new Gate();
    private final FlowerRemover flowerRemover = new FlowerRemover();

    private final Pose startPose = poseFactory.of(56, 8, 180);
    private final Pose preGarden = poseFactory.of(8, 16, 270);
    private final Pose garden = poseFactory.of(8, 9, 270);
    private final Pose score2 = poseFactory.of(33, 130, 270);
    private final Pose preFlower1 = poseFactory.of(47, 115, 90);
    private final Pose flower1 = poseFactory.of(47, 125, 90);
    private final Pose park = poseFactory.of(15, 121, 0);

    private Path startToPreGarden() {
        return line(startPose, preGarden).linear(startPose, preGarden);
    }

    private Path preGardenToGarden() {
        return line(preGarden, garden).linear(preGarden, garden);
    }

    private Path gardenToScore2() {
        return line(garden, score2).linear(garden, score2);
    }

    private Path score2ToPreFlower1() {
        return line(score2, preFlower1).linear(score2, preFlower1);
    }

    private Path preFlower1ToFlower1() {
        return line(preFlower1, flower1).linear(preFlower1, flower1);
    }

    private Path flower1ToPark() {
        return line(flower1, park).linear(flower1, park);
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, startToPreGarden()),
                follow(follower, preGardenToGarden()),
                follow(follower, gardenToScore2()),
                follow(follower, score2ToPreFlower1()),
                follow(follower, preFlower1ToFlower1()),
                follow(follower, flower1ToPark())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) {
            PoseManager.save(follower.pose());
        }
    }
}