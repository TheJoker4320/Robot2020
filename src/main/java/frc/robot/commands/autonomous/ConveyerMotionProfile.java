// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.robot.commands.autonomous;

// import java.util.List;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.controller.PIDController;
// import edu.wpi.first.wpilibj.controller.RamseteController;
// import edu.wpi.first.wpilibj.geometry.Pose2d;
// import edu.wpi.first.wpilibj.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
// import edu.wpi.first.wpilibj.trajectory.Trajectory;
// import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
// import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
// import frc.robot.subsystems.MainConveyer;

// /**
//  * An example command that uses an example subsystem.
//  */
// public class ConveyerMotionProfile extends CommandBase {
//     // @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
//     private final MainConveyer mainConveyer;
//     private final Timer m_timer = new Timer();
//     private DifferentialDriveKinematics kinematics;
//     private RamseteController ramseteFollower;
//     private Trajectory trajectory;
//     private PIDController pidController;
//     public double ballsAmount = 0;

//     public ConveyerMotionProfile(MainConveyer mainConveyer, double ballsAmount) {
//         this.ballsAmount = ballsAmount;
//         kinematics = new DifferentialDriveKinematics(0);
//         TrajectoryConfig config = new TrajectoryConfig(Constants.Constraints.MainConveyer.maxVelocity,
//                 Constants.Constraints.MainConveyer.maxAcceleration);
//                         //.addConstraint(Constants.Constraints.MainConveyer.voltageConstraint);
//         trajectory = TrajectoryGenerator.generateTrajectory(
//                 List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(setPoint(), 0, new Rotation2d(0))), config);
//         pidController = new PIDController(Constants.PIDBZ.MainConveyer.Kp, Constants.PIDBZ.MainConveyer.Ki,
//                 Constants.PIDBZ.MainConveyer.Kd);
//         ramseteFollower = new RamseteController(Constants.PIDBZ.MainConveyer.B, Constants.PIDBZ.MainConveyer.Zeta);
//         this.mainConveyer = mainConveyer;
//         //addRequirements(conveyer);
//     }

//     private double setPoint() {
//         return this.ballsAmount * Constants.Robot.POWER_CELL_DIAMETER;
//     }

//     public double ballsConveyered() {
//         return mainConveyer.distanceInMeters() / Constants.Robot.POWER_CELL_DIAMETER;
//     }

//     // Called when the command is initially scheduled.
//     @Override
//     public void initialize() {
//         mainConveyer.resetDistance();
//         m_timer.reset();
//         m_timer.start();
//         pidController.reset();
//     }

//     // Called every time the scheduler runs while the command is scheduled.
//     @Override
//     public void execute() {
//         double curTime = m_timer.get();
//         System.out.println("Trajectory: "+trajectory.sample(curTime));
//         var targetWheelSpeeds = kinematics.toWheelSpeeds(ramseteFollower.calculate(
//                 new Pose2d(this.mainConveyer.distanceInMeters(), 0, new Rotation2d()), trajectory.sample(curTime)));

//         var SpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
//         System.out.println("Speed setpoint: "+SpeedSetpoint);
//         double output = pidController.calculate(this.mainConveyer.velocityInMeters(), SpeedSetpoint);
//         System.out.println("Output: "+output);
//         this.mainConveyer.setOutput(output);
//     }

//     // Called once the command ends or is interrupted.
//     @Override
//     public void end(boolean interrupted) {
//         this.mainConveyer.setOutput(0);
//         m_timer.stop();
//     }

//     // Returns true when the command should end.
//     @Override
//     public boolean isFinished() {
//         return m_timer.hasPeriodPassed(trajectory.getTotalTimeSeconds());
//     }
// }
