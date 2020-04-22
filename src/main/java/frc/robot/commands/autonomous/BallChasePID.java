package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis;

public class BallChasePID extends CommandBase {
    private Chassis chassis;

    public PIDController leftController;
    public PIDController rightController;
    public PIDController turnController;

    public BallChasePID(Chassis chassis) {
        this.chassis = chassis;
        addRequirements(chassis);
        leftController = new PIDController(1, 0, 0);
        leftController.setTolerance(0.1);
        rightController = new PIDController(1, 0, 0);
        rightController.setTolerance(0.1);
        turnController = new PIDController(0.2, 0, 0);
        turnController.setTolerance(5);
    }

    @Override
    public void initialize() {
        leftController.reset();
        rightController.reset();
        turnController.reset();
    }

    public double updateDistance() {
        if (chassis.getDistanceFromBall() > 0.1) {
            chassis.resetEncoders();
        }
        return chassis.getDistanceFromBall();
    }

    public double updateAngle() {
        if (chassis.getAngleFromBall() > 5) {
            chassis.zeroHeading();
        }
        return chassis.getAngleFromBall();
    }

    @Override
    public void execute() {
        double distance = this.updateDistance();
        double angle = this.updateAngle();
        chassis.setOutput(
                leftController.calculate(chassis.getLeftDistance(), distance)
                        - turnController.calculate(chassis.getHeading(), angle),
                rightController.calculate(chassis.getRightDistance(), distance)
                        + turnController.calculate(chassis.getHeading(), angle));
    }

    @Override
    public boolean isFinished() {
        return leftController.atSetpoint() && rightController.atSetpoint() && turnController.atSetpoint();

    }

    @Override
    public void end(final boolean interrupted) {
        chassis.tankDrive(0, 0);
    }

}