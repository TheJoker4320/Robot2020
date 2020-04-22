package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Crank extends SubsystemBase {
  private static Crank instance;
  private WPI_TalonSRX masterMotor;

  public static Crank getInstance() {
    if (instance == null)
      instance = new Crank();
    return instance;
  }

  private Crank() {
    masterMotor = new WPI_TalonSRX(ClimberConstants.CRANK_MOTOR);
    masterMotor.configFactoryDefault();

  }

  public void setOutput(double percentage) {
    masterMotor.set(percentage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
