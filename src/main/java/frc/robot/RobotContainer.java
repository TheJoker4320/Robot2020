/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Hashtable;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Climb;
import frc.robot.commands.CompressorToggle;
import frc.robot.commands.Convey;
import frc.robot.commands.ManualShoot;
import frc.robot.commands.MoveCrank;
import frc.robot.commands.autonomous.EmergencyConveyer;
import frc.robot.commands.autonomous.PIDShooter;
import frc.robot.commands.ShiftPowerMode;
import frc.robot.commands.GripperToggle;
import frc.robot.commands.IntakeLifterToggle;
import frc.robot.commands.time_automated.AutomatedChassis;
import frc.robot.commands.time_automated.AutomatedMainConveyer;
import frc.robot.commands.time_automated.AutomatedShooter;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.PowerCellCounter;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CompressorWrapper;
import frc.robot.subsystems.Crank;
import frc.robot.subsystems.IntakeGripper;
import frc.robot.subsystems.IntakeLifter;
import frc.robot.subsystems.JokerSubsystem;
import frc.robot.subsystems.MainConveyer;
import frc.robot.subsystems.Shifters;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  //public static Crank crank = Crank.getInstance();
  //public static Climber climber = Climber.getInstance();
  public static Hashtable<String, JokerSubsystem> subsystems = new Hashtable<String, JokerSubsystem>();
  public static IntakeGripper intakeGripper = IntakeGripper.getInstance();
  public static IntakeLifter intakeLifter = IntakeLifter.getInstance();
  public static MainConveyer mainConveyer = MainConveyer.getInstance();
  public static Shooter shooter = Shooter.getInstance();
  public static Chassis chassis = Chassis.getInstance();
  public static Shifters shifters = Shifters.getInstance();
  public static CompressorWrapper compressorWrapper = CompressorWrapper.getInstance();
  public Joystick leftJoystick;
  public  XboxController controller;
  private CommandBase m_autoCommand;

  private JoystickButton emergencyButton;
  //private POVButton up, down;
  // private final ConveyerMotionProfile m_autoCommand;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    subsystems.put("intakeGripper", IntakeGripper.getInstance());
    subsystems.put("intakeLifter", IntakeLifter.getInstance());
    subsystems.put("mainConveyer", MainConveyer.getInstance());
    subsystems.put("shooter", Shooter.getInstance());
    //subsystems.put("shifters", Shifters.getInstance()); Can't change shifters state when not in motion
    subsystems.put("compressorWrapper", CompressorWrapper.getInstance());


    leftJoystick = new Joystick(Constants.OIConstants.LEFT_JOYSTICK_PORT);
    controller = new XboxController(1);

    chassis.setDefaultCommand(new TeleopDrive(chassis, () -> this.leftJoystick.getY(), () -> this.leftJoystick.getZ()));
    intakeGripper.setDefaultCommand(new GripperToggle(intakeGripper, () -> controller.getRawButton(1),
                                    () -> controller.getTriggerAxis(Hand.kLeft)));
    intakeLifter.setDefaultCommand(new IntakeLifterToggle(intakeLifter, ()->controller.getRawButtonReleased(4)));
    mainConveyer.setDefaultCommand(new Convey(mainConveyer,
        () -> this.controller.getRawButton(Constants.OIConstants.MAIN_CONVEYER_BUTTON_INSIDE),
        () -> this.controller.getRawButton(Constants.OIConstants.MAIN_CONVEYER_BUTTON_OUTSIDE)));
    shooter.setDefaultCommand(new ManualShoot(shooter, ()->controller.getBumperReleased(Hand.kRight)));
    shifters.setDefaultCommand(new ShiftPowerMode(shifters, ()->leftJoystick.getRawButton(11)));  
    compressorWrapper.setDefaultCommand(new CompressorToggle(compressorWrapper, ()->controller.getBumperReleased(Hand.kLeft)));
    //shooter.setDefaultCommand(new PIDShooter(shooter, 60));

    //up = new POVButton(controller, 0);
    //down = new POVButton(controller, 180);    
    //crank.setDefaultCommand(new MoveCrank(crank, ()->controller.getBackButtonPressed()));
    
    //climber.setDefaultCommand(new Climb(climber, () -> up.get(), () -> down.get()));
    // //m_autoCommand = new ConveyerMotionProfile(mainConveyer, 1);
    // /*
    // m_autoCommand = new PIDCommand(new PIDController(20, 0, 0.5), ()->mainConveyer.distanceInMeters(),
    // 0.25, (double a)->mainConveyer.setOutput(a), mainConveyer);
    // */
    /*
    m_autoCommand = new ParallelCommandGroup(new AutomatedShooter(shooter, true, 10), 
      new SequentialCommandGroup(new SequentialCommandGroup(new WaitCommand(2),
        //new AutomatedShooterConveyer(shooterConveyer, true, 0.2), 
        new ParallelCommandGroup(new AutomatedShooterConveyer(shooterConveyer, true, 2.5), 
        new AutomatedMainConveyer(mainConveyer, true, 1))), new WaitCommand(2),
              new SequentialCommandGroup(new SequentialCommandGroup(
              //new AutomatedShooterConveyer(shooterConveyer, true, 0.2), 
              new ParallelCommandGroup(new AutomatedShooterConveyer(shooterConveyer, true, 2.5), 
              new AutomatedMainConveyer(mainConveyer, true, 1)))), new WaitCommand(2),
                      new SequentialCommandGroup(new SequentialCommandGroup(
                      //new AutomatedShooterConveyer(shooterConveyer, true, 0.2), 
                      new ParallelCommandGroup(new AutomatedShooterConveyer(shooterConveyer, true, 2.5), 
                      new AutomatedMainConveyer(mainConveyer, true, 1)))
                      //new AutomatedChassis(chassis, true, 1.5)
                      )));
                      */
    //m_autoCommand = new AutomatedChassis(chassis, true, 2);
    m_autoCommand = new PIDShooter(shooter, 20);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    emergencyButton = new JoystickButton(controller, 12);
    emergencyButton.whenHeld(new EmergencyConveyer(mainConveyer, intakeGripper, true, 1), true);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //mainConveyer.resetDistance();
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
