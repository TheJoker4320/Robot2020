package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis;
import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.lang.Math;

public class TeleopDrive extends CommandBase {
    private Chassis chassis;
    // Drive constatns
    protected double parabolaA = 1;
    protected double exponent = 1;

    // Drive speeds and triggers
    protected DoubleSupplier leftSpeed;
    protected DoubleSupplier rightSpeed;
    protected DoubleSupplier ySpeed;
    protected DoubleSupplier zSpeed;

    protected BooleanSupplier leftTrigger;
    protected BooleanSupplier rightTrigger;

    public TeleopDrive(Chassis chassis, DoubleSupplier ySpeed, DoubleSupplier zSpeed) {
        addRequirements(chassis);
        this.chassis = chassis;
        this.ySpeed = ySpeed;
        this.zSpeed = zSpeed;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        this.exponent = SmartDashboard.getNumber("Joystick Sensitivity", 3);
        this.exponent = this.exponent * (this.exponent + 1);
        chassis.drive.arcadeDrive(ySpeed.getAsDouble(), -zSpeed.getAsDouble());
        //chassis.arcadeDrive(exponentialDrive(this.ySpeed.getAsDouble()),exponentialDrive(-this.zSpeed.getAsDouble()));
    }

    public double exponentialDrive(double speed) {
        if (speed >= 0)
            return ((Math.pow(this.exponent, speed) - 1) / (this.exponent - 1));

        else
            return -((Math.pow(this.exponent, -speed) - 1) / (this.exponent - 1));
    }

    public double parabolicDrive(double joystick) {
        if (joystick >= 0)
            return Math.pow(joystick, 2) * this.parabolaA;
        else
            return -Math.pow(joystick, 2) * this.parabolaA;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        chassis.tankDrive(0, 0);
    }
}