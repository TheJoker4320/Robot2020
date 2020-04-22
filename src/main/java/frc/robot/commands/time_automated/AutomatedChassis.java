package frc.robot.commands.time_automated;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis;
import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.lang.Math;

public class AutomatedChassis extends CommandBase {
    private Chassis chassis;
    private boolean work;
    private double time;
    private Timer timer;


    public AutomatedChassis(Chassis chassis, boolean work, double time){
        this.work = work;
        this.time = time;
        this.chassis = chassis;
        this.timer = new Timer();
        addRequirements(chassis);
        this.chassis = chassis;
    }

    @Override
    public void initialize() {
        this.timer.reset();
        this.timer.start();
    }

    @Override
    public void execute() {
        if(work)
            chassis.drive.arcadeDrive(-0.7, 0);
    }

    @Override
    public boolean isFinished() {
        return timer.get() > time;    }

    @Override
    public void end(boolean interrupted) {
        chassis.drive.arcadeDrive(0, 0);
    }
}