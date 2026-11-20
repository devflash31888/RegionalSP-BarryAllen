package org.firstinspires.ftc.teamcode.OpModesRegional.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystemsRegional.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystemsRegional.FlowerRemover;
import org.firstinspires.ftc.teamcode.SubSystemsRegional.Gate;
import org.firstinspires.ftc.teamcode.SubSystemsRegional.Intake;
import org.firstinspires.ftc.teamcode.Tuning.ShooterSubSystem;

import dev.easyfamily.EasyStack.components.EasyCommandComponent;
import dev.easyfamily.EasyStack.components.EasySubsystemComponent;
import dev.easyfamily.EasyStack.gamepad.EasyGamepad;
import dev.easyfamily.EasyStack.opmode.EasyOpMode;

@TeleOp(name = "Teleop Regional Shooter Tuning")
public class TeleopTuningShooter extends EasyOpMode {

    private final Intake intake = new Intake();
    private final Gate gate = new Gate();
    private final DriveTrain drivetrain = new DriveTrain();
    private final FlowerRemover flower = new FlowerRemover();
    private final ShooterSubSystem shooter = new ShooterSubSystem();
    private EasyGamepad score, nav;

    private double tuningKS = 0.025;
    private double tuningKV = 0.00042;
    private double tuningKP = 0.0045;
    private double targetVelocity = 0.0;
    private int selectedParam = 0; // 0 = kS, 1 = kV, 2 = kP

    @Override
    public void initialize() {
        nav = new EasyGamepad(gamepad1);
        score = new EasyGamepad(gamepad2);

        addComponents(
                new EasySubsystemComponent(intake),
                new EasySubsystemComponent(gate),
                new EasySubsystemComponent(drivetrain),
                new EasySubsystemComponent(flower),
                new EasySubsystemComponent(shooter),
                new EasyCommandComponent()
        );
    }

    @Override
    public void onUpdate() {
        nav.update();
        score.update();

        drivetrain.teleopDrive(nav);

        if (nav.xPressed()) {
            drivetrain.resetHeading();
        }

        if (nav.aPressed()) {
            drivetrain.relocalize();
        }

        if (score.x()) {
            intake.turnReverse();
        } else if (score.a()) {
            gate.open();

            if (gate.hasWaitTimeElapsed()) {
                intake.turnOn();
            }
        } else {
            gate.close();

            if (score.yToggle()) {
                intake.turnOn();
            } else {
                intake.turnOff();
            }
        }

        if (score.bToggle()) {
            flower.down();
        } else {
            flower.up();
        }

        // Alterna entre kS, kV e kP usando os Bumpers do gamepad2 (score)
        if (score.rightBumperPressed()) {
            selectedParam = (selectedParam + 1) % 3;
        }
        if (score.leftBumperPressed()) {
            selectedParam = (selectedParam + 2) % 3;
        }

        // Aumenta o ganho selecionado com D-Pad Cima
        if (score.dpadUpPressed()) {
            if (selectedParam == 0) tuningKS += 0.005;
            else if (selectedParam == 1) tuningKV += 0.00002;
            else if (selectedParam == 2) tuningKP += 0.0005;
        }
        // Diminui o ganho selecionado com D-Pad Baixo (travando em 0.0)
        if (score.dpadDownPressed()) {
            if (selectedParam == 0) tuningKS = Math.max(0.0, tuningKS - 0.005);
            else if (selectedParam == 1) tuningKV = Math.max(0.0, tuningKV - 0.00002);
            else if (selectedParam == 2) tuningKP = Math.max(0.0, tuningKP - 0.0005);
        }

        // Aumenta ou diminui a velocidade alvo do Shooter usando D-Pad Direita/Esquerda
        if (score.dpadRightPressed()) {
            targetVelocity += 100.0;
        }
        if (score.dpadLeftPressed()) {
            targetVelocity = Math.max(0.0, targetVelocity - 100.0);
        }

        shooter.setGains(tuningKP, tuningKS, tuningKV);
        shooter.setTargetVelocity(targetVelocity);

        String[] paramNames = {"kS (Atrito Estático)", "kV (Velocidade)", "kP (Proporcional)"};

        telemetry.addData("Posição X", drivetrain.getPose().x());
        telemetry.addData("Posição Y", drivetrain.getPose().y());
        telemetry.addData("Heading", drivetrain.getPose().heading());
        telemetry.addLine("--- SHOOTER TUNING ---");
        telemetry.addData("Shooter Alvo (RPM)", shooter.getTargetVelocity());
        telemetry.addData("Shooter A Atual (RPM)", String.format("%.1f", shooter.getCurrentVelocityA()));
        telemetry.addData("Shooter B Atual (RPM)", String.format("%.1f", shooter.getCurrentVelocityB()));
        telemetry.addData("Editando", paramNames[selectedParam]);
        telemetry.addData("Ganhos", String.format("kS: %.4f | kV: %.6f | kP: %.5f", tuningKS, tuningKV, tuningKP));
        telemetry.update();
    }

    @Override
    public void onStop() {
        intake.turnOff();
        gate.close();
        shooter.stop();
    }
}