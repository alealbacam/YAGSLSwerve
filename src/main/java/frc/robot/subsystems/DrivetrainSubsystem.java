package frc.robot.subsystems;


import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;


public class DrivetrainSubsystem extends SubsystemBase
{
   private final SwerveDrive swerveDrive;


   public DrivetrainSubsystem(File directory)
   {
      try{
       swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive(4.18);
       }catch(IOException ex){
           throw new RuntimeException(ex);
       }
       SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
      
   }


   public DrivetrainSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg)
   {
   swerveDrive = new SwerveDrive(driveCfg, controllerCfg, 4.18);
   }


  


     /**
  * Command to drive the robot using translative values and heading as a setpoint.
  *
  * @param translationX Translation in the X direction.
  * @param translationY Translation in the Y direction.
  * @param headingX     Heading X to calculate angle of the joystick.
  * @param headingY     Heading Y to calculate angle of the joystick.
  * @return D rive command.
  */
   public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                             DoubleSupplier headingY)
 {
   return run(() -> {
     double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth controll out
     double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth controll out
     // Make the robot move
     driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput, yInput,
                                                                     headingX.getAsDouble(),
                                                                     headingY.getAsDouble(),
                                                                     swerveDrive.getYaw().getRadians(),
                                                                     swerveDrive.getMaximumVelocity()));
   });
 }


 public ChassisSpeeds getFieldVelocity()
 {
   return swerveDrive.getFieldVelocity();
 }


 public SwerveDriveConfiguration getSwerveDriveConfiguration()
 {
   return swerveDrive.swerveDriveConfiguration;
 }






 public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
 {
   xInput = Math.pow(xInput, 3);
   yInput = Math.pow(yInput, 3);
   return swerveDrive.swerveController.getTargetSpeeds(xInput,
                                                       yInput,
                                                       headingX,
                                                       headingY,
                                                       getHeading().getRadians(),
                                                       4.18);
 }


  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
 {
   xInput = Math.pow(xInput, 3);
   yInput = Math.pow(yInput, 3);
   return swerveDrive.swerveController.getTargetSpeeds(xInput,
                                                       yInput,
                                                       angle.getRadians(),
                                                       getHeading().getRadians(),
                                                       4.18);
 }


 public Rotation2d getHeading()
 {
   return getPose().getRotation();
 }


  public Pose2d getPose()
 {
   return swerveDrive.getPose();
 }


 public void drive(Translation2d translation, double rotation, boolean fieldRelative)
 {
   swerveDrive.drive(translation,
                     rotation,
                     fieldRelative,
                     false); // Open loop is disabled since it shouldn't be used most of the time.
 }


 public void drive(ChassisSpeeds velocity)
 {
   swerveDrive.drive(velocity);
 }

 public void addFakeVisionReading()
 {
   swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
 }


  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }
 
 public void driveFieldOriented(ChassisSpeeds velocity){
   swerveDrive.driveFieldOriented(velocity);
 }

 public Command simDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation)
 {
   // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
   return run(() -> {
     // Make the robot move
     driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(translationX.getAsDouble(),
                                                                     translationY.getAsDouble(),
                                                                     rotation.getAsDouble() * Math.PI,
                                                                     swerveDrive.getOdometryHeading().getRadians(),
                                                                     swerveDrive.getMaximumVelocity()));
   });
 }

 public void setMotorBrake(boolean brake)
 {
   swerveDrive.setMotorIdleMode(brake);
 }

 /**
  * Command to drive the robot using translative values and heading as angular velocity.
  *
  * @param translationX     Translation in the X direction.
  * @param translationY     Translation in the Y direction.
  * @param angularRotationX Rotation of the robot to set
  * @return Drive command.
  */
 public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
 {
   return run(() -> {
     // Make the robot move
     swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumVelocity(),
                                         translationY.getAsDouble() * swerveDrive.getMaximumVelocity()),
                       angularRotationX.getAsDouble() * swerveDrive.getMaximumAngularVelocity(),
                       true,
                       false);
   });
  
 }


}
