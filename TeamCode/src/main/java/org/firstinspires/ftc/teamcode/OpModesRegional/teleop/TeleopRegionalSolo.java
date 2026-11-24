package org.firstinspires.ftc.teamcode.OpModesRegional.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystemsRegional.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystemsRegional.FlowerRemover;
import org.firstinspires.ftc.teamcode.SubSystemsRegional.Gate;
import org.firstinspires.ftc.teamcode.SubSystemsRegional.Intake;
import org.firstinspires.ftc.teamcode.SubSystemsRegional.Shooter;
import org.firstinspires.ftc.teamcode.SubSystemsRegional.Turret;

import dev.easyfamily.EasyStack.components.EasyCommandComponent;
import dev.easyfamily.EasyStack.components.EasySubsystemComponent;
import dev.easyfamily.EasyStack.gamepad.EasyGamepad;
import dev.easyfamily.EasyStack.opmode.EasyOpMode;

@TeleOp(name = "Teleop Regional Solo")
public class TeleopRegionalSolo extends EasyOpMode {

    private final Intake intake = new Intake();
    private final Gate gate = new Gate();
    private final DriveTrain drivetrain = new DriveTrain();
    private final FlowerRemover flower = new FlowerRemover();
    private final Turret turret = new Turret();
    private final Shooter shooter = new Shooter();

    private EasyGamepad score, nav;

    @Override
    public void initialize() {
        nav = new EasyGamepad(gamepad1);
        score = new EasyGamepad(gamepad2);

        addComponents(
                new EasySubsystemComponent(intake),
                new EasySubsystemComponent(gate),
                new EasySubsystemComponent(drivetrain),
                new EasySubsystemComponent(flower),
                new EasySubsystemComponent(turret),
                new EasySubsystemComponent(shooter),
                new EasyCommandComponent()
        );
    }

    @Override
    public void onUpdate() {
        nav.update();
        score.update();

        drivetrain.teleopDrive(nav);

        if (nav.dpadDownPressed()) {
            drivetrain.resetHeading();
        }

        if (nav.dpadUpPressed()) {
            drivetrain.relocalize();
        }

        double robotX = drivetrain.getPose().x();
        double robotY = drivetrain.getPose().y();
        double robotHeading = drivetrain.getHeading();

        if (nav.startPressed()) {
            turret.setManualTarget();
            shooter.setDefault();
        } else {
            turret.aimToGoal(robotX, robotY, robotHeading, drivetrain.getXVelocity(), drivetrain.getYVelocity());
            shooter.calculateDistanceAndInterpolate(robotX, robotY);

            if (nav.rightBumperPressed()) {
                turret.incrementManualOffset();
            } else if (nav.leftBumperPressed()) {
                turret.decrementManualOffset();
            }
        }

        if (nav.x()) {
            intake.turnReverse();
        } else if (nav.a()) {
            gate.open();

            if (gate.hasWaitTimeElapsed()) {
                intake.turnOn();
            }
        } else {
            gate.close();

            if (nav.yToggle()) {
                intake.turnOn();
            } else {
                intake.turnOff();
            }
        }

        if (nav.bToggle()) {
            flower.down();
        } else {
            flower.up();
        }

        telemetry.addData("Turret Alvo", turret.getTargetDegrees());
        telemetry.addData("Turret Offset Manual", turret.getManualOffset());
        telemetry.addData("Posição X", robotX);
        telemetry.addData("Posição Y", robotY);
        telemetry.addData("Posição Z", robotHeading);
        telemetry.addData("Shooter Target LUT", shooter.getTargetVelocity());
        telemetry.update();
    }

    @Override
    public void onStop() {
        intake.turnOff();
        gate.close();
        turret.stop();
        shooter.stop();
    }
}