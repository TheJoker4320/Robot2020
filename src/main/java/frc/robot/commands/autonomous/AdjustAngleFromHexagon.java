package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
//import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Chassis;


public class AdjustAngleFromHexagon extends PIDCommand {

  public AdjustAngleFromHexagon(Chassis subsystem, double targetAngleDegrees) {
    super(new PIDController(1, 0, 0),
        subsystem::getAngleFromHexagon,
        targetAngleDegrees,
        output -> subsystem.setOutput(output, -output),
        subsystem);

    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(Constants.Robot.TURN_TOLERANCE_DEG, Constants.Robot.TURN_RATE_TOLERANCE_DEG_PERS);
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}