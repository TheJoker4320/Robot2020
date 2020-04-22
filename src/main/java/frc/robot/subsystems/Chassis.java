/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants.DriveConstants;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class Chassis extends SubsystemBase {

  public static Chassis instance;

  private WPI_TalonSRX leftMaster;
  private WPI_VictorSPX leftSlave;
  private WPI_TalonSRX rightMaster;
  private WPI_VictorSPX rightSlave;
  private AHRS navx;

  public DifferentialDrive drive;
  public NetworkTableInstance networkTable;
  public Pose2d pose;
  public boolean active=false;
  public double max_voltage = 12;
  // private double angle = 1.0;

  public static Chassis getInstance() {
    if(instance==null)
      instance=new Chassis();
    return instance;
  }

  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(
      Rotation2d.fromDegrees(getHeading()));

  private Chassis() {
    leftMaster = new WPI_TalonSRX(2);
    leftMaster.configFactoryDefault();
    leftSlave = new WPI_VictorSPX(1);
    leftSlave.configFactoryDefault();
    rightMaster = new WPI_TalonSRX(4);
    rightMaster.configFactoryDefault();
    rightSlave = new WPI_VictorSPX(3);

    navx = new AHRS(SPI.Port.kMXP);

    this.networkTable = NetworkTableInstance.getDefault();

    leftMaster.setInverted(false);
    leftSlave.setInverted(true);
    rightMaster.setInverted(false);
    rightSlave.setInverted(false);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    /*
    leftMaster.setInverted(true);
    leftSlave.setInverted(true);
    rightMaster.setInverted(false);
    rightSlave.setInverted(false);

    leftMaster.setSensorPhase(true);
    rightMaster.setSensorPhase(true);
    */
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    

    
    leftMaster.configVoltageCompSaturation(max_voltage);
    rightMaster.configVoltageCompSaturation(max_voltage);
    leftMaster.enableVoltageCompensation(false);
    rightMaster.enableVoltageCompensation(false);
    

    drive = new DifferentialDrive(leftMaster, rightMaster);

  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void arcadeDrive(double xSpeed, double zRotation) {
    //setOutput(4, 4);
    drive.arcadeDrive(xSpeed, zRotation);
  }

  public double getDistanceFromBall() {
    return Math.sqrt(Math.pow(networkTable.getEntry("distance_vector_x").getDouble(0), 2)
        + Math.pow(networkTable.getEntry("distance_vector_z").getDouble(0), 2));
  }

  public double getAngleFromBall() {
    return this.networkTable.getEntry("x_angle").getDouble(0);
  }

  public double getAngleFromHexagon() {
    return this.networkTable.getEntry("x_angle").getDouble(0);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getDistanceInMeters(leftMaster.getSelectedSensorVelocity() * 10),
        getDistanceInMeters(rightMaster.getSelectedSensorVelocity() * 10));
  }

  public void resetEncoders() {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  public double getHeading() {
    if(navx != null && navx.isConnected())
      return Math.IEEEremainder(-navx.getYaw(), 180) * (Constants.Robot.GYRO_REVERSED ? -1.0 : 1.0);
    return 0;

  }

  public Pose2d getPose() {
    return pose;
  }


  public double getDistanceInMeters(double encoderTicks) {
    return Constants.Robot.ENCODER_DISTANCE_PER_PULSE * encoderTicks;
  }

  public double getLeftDistance() {
    return getDistanceInMeters(leftMaster.getSelectedSensorPosition());
  }

  public double getRightDistance() {
    return getDistanceInMeters(rightMaster.getSelectedSensorPosition());
  }

  
  public double getTurnRate() {
    return navx.getRate() * (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
  }

  public void setOutput(double leftPower, double rightPower) {
    this.leftMaster.set(leftPower/max_voltage);
    this.rightMaster.set(rightPower/max_voltage);
    /*
    if (leftPower < 0) {
      leftMaster.setVoltage(leftPower);
      rightMaster.setVoltage(rightPower + (1 - Math.cos(Math.toRadians(this.angle))) * leftPower);
    } else {
      leftMaster.setVoltage(leftPower);
      rightMaster.setVoltage(rightPower - (1 - Math.cos(Math.toRadians(this.angle))) * leftPower);
    }
    */
  }

  public void zeroHeading() { this.navx.zeroYaw(); }

  @Override
  public void periodic() {
    /*
    System.out.println("Left encoder: " + leftMaster.getSelectedSensorPosition() + " Right encoder: "
        + rightMaster.getSelectedSensorPosition());
    pose = odometry.update(Rotation2d.fromDegrees(getHeading()),
        getDistanceInMeters(leftMaster.getSelectedSensorPosition()),
        getDistanceInMeters(rightMaster.getSelectedSensorPosition()));
        */
  }

}
