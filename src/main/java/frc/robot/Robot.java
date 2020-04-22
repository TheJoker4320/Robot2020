/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.IntakeGripper;
import frc.robot.subsystems.JokerSubsystem;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.cameraserver.CameraServer;



public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private UsbCamera camera1;
  private UsbCamera camera2;
  // private CvSink cvSink;
  // private CvSource outputStream;
  private VideoSink cameraServer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {    
    SmartDashboard.putBoolean("Shooter Range", false);

    SmartDashboard.putBoolean("stream_camera1", false);
    // this.cameraServer = CameraServer.getInstance().getServer();
    // this.cameraServer = CameraServer.getInstance().getServer();
    // this.mjpegServer = new MjpegServer("trapez", "http://tj-nano:5000/trapez",
    // 5000);

    this.camera1 = CameraServer.getInstance().startAutomaticCapture(0);
    // this.camera2 = new HttpCamera("trapez", "url");
    // this.camera3 = new HttpCamera("balls", "url");
    int imageWidth = 240;
    int imageHeight = 180;

    this.camera1.setResolution(imageWidth, imageHeight);
    this.camera2 = CameraServer.getInstance().startAutomaticCapture(1);
    this.camera2.setResolution(180, 120);

    // this.camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    // this.camera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    // this.camera3.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    // this.camera2.setResolution(imageWidth,imageHeight);
    // this.camera3.setResolution(imageWidth,imageHeight);
    /*
     * new Thread(() -> {
     * 
     * CvSink cvSink = CameraServer.getInstance().getVideo(camera1); CvSource
     * outputStream = CameraServer.getInstance().putVideo("image", imageWidth,
     * imageHeight);
     * 
     * Mat source = new Mat(); Mat output = new Mat();
     */
    /*
     * while (!Thread.interrupted()){
     * 
     * 
     * //if (SmartDashboard.getEntry("stream_camera1").getBoolean(false)) {
     * //this.cameraServer = CameraServer.getInstance().getServer();
     * //this.cameraServer.setSource(this.camera1);
     * 
     * /* if (SmartDashboard.getEntry("stream_camera2").getBoolean(false)){
     * this.cameraServer.setSource(this.camera2); }
     * 
     * if (SmartDashboard.getEntry("stream_camera3").getBoolean(false)){
     * this.cameraServer.setSource(this.camera3); }
     */
    /*
     * cvSink.grabFrame(source);
     * 
     * Point pt1_h = new Point(imageWidth/2 - 20, imageHeight/2); Point pt2_h = new
     * Point(imageWidth/2 + 20, imageHeight/2);
     * 
     * Point pt1_v = new Point(imageWidth/2, imageHeight/2 - 20); Point pt2_v = new
     * Point(imageWidth/2, imageHeight/2 + 20);
     * 
     * Imgproc.line(source, pt1_h, pt2_h, new Scalar(0,255,0), 3);
     * Imgproc.line(source , pt1_v, pt2_v, new Scalar(0,255,0), 3);
     * 
     * outputStream.putFrame(source); //}
     * 
     * } }).start();
     */
    // this.compressor = new Compressor();
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if (Math.sqrt(Math.pow(SmartDashboard.getNumber("trapez_distance_vector_z", 0), 2)
            + Math.pow(SmartDashboard.getNumber("trapez_distance_vector_z", 0), 2)) >= 55
        && Math.sqrt(Math.pow(SmartDashboard.getNumber("trapez_distance_vector_z", 0), 2)
            + Math.pow(SmartDashboard.getNumber("trapez_distance_vector_z", 0), 2)) <= 85) 
    {
      SmartDashboard.putBoolean("Shooter Range", true);
    } 
    else {
      SmartDashboard.putBoolean("Shooter Range", false);
    }

    if(m_robotContainer.intakeGripper.isActive() && m_robotContainer.intakeLifter.isActive()){
      m_robotContainer.intakeLifter.disable();
    }

  }
  
  @Override
  public void disabledInit() {
    for (JokerSubsystem subsystem : RobotContainer.subsystems.values()) {
      subsystem.disable();
    }
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
}
