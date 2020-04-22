package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Chassis;

public class RamsetDrive extends CommandBase {
    private Chassis chassis;
    protected Trajectory trajectory;
    public RamseteCommand command;
    public Double kP;
    public Double kI;
    public Double kD;

    public RamsetDrive(Chassis chassis, Trajectory trajectory, Double kP, Double kI, Double kD) {
        this.chassis = chassis;
        addRequirements(chassis);
        this.trajectory = trajectory;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    @Override
    public void initialize() {
        chassis.zeroHeading();
        chassis.resetEncoders();
        this.command = new RamseteCommand(trajectory, chassis::getPose,
                new RamseteController(Constants.PIDBZ.Chassis.B, Constants.PIDBZ.Chassis.Zeta),
                new SimpleMotorFeedforward(Constants.PIDBZ.Chassis.Ks, Constants.PIDBZ.Chassis.Kv,
                        Constants.PIDBZ.Chassis.Ka),
                DriveConstants.kDriveKinematics, chassis::getWheelSpeeds, new PIDController(kP, kI, kD),
                new PIDController(kP, kI, kD), chassis::setOutput, chassis);
        this.command.initialize();
    }

    @Override
    public void execute() {
        //try motion profiling with feedback from camera
        command.execute();
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

    @Override
    public void end(final boolean interrupted) {
        command.end(interrupted);
        chassis.tankDrive(0, 0);
    }
}