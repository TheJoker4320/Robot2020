package frc.robot.commands.autonomous;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class PIDShooter extends CommandBase {
    // @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private Shooter shooter;
    public BooleanSupplier toggle;
    private PIDController controller;
    private SimpleMotorFeedforward feedForward;
    private double speed;

    public PIDShooter(Shooter shooter, double speed) {
        controller = new PIDController(0.4, 0, 0);
        feedForward = new SimpleMotorFeedforward(5, 0, 0);
        this.speed = speed;
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        controller.enableContinuousInput(-100, 100);
        controller.setTolerance(0.01);
        controller.setSetpoint(speed);
    }

    @Override
    public void execute() {
        shooter.setVoltage(controller.calculate(shooter.velocityInMeters())+feedForward.calculate(shooter.velocityInMeters()));
    }

    @Override
    public void end(boolean interrupted) {
        shooter.disable();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
