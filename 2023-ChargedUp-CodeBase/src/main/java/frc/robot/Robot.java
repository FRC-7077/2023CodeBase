/* 
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
*/

package frc.robot;

// import for use with Acceleration Curve system
import java.util.ArrayList;

// import for the navX2-micro 9-axis Gyro
import com.kauailabs.navx.frc.AHRS;

// Standard imports from WPILib for hardware and SmartDashboard
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Servo;

// Rev Robotics imports for SparkMax controllers
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

// WPILib imports for NetworkTable access with Limelight
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot
{
  /*
   * The order of these methods is so that the commonly adjusted methods are up top
   * and the methods you should never need to adjust are below them to reduce scrolling
   */
  

   // ******************  AUTONOMOUS CODE  **************************************************************************
  @Override
  public void autonomousInit() 
  {
    leftMotor1.setIdleMode(IdleMode.kBrake);
    leftMotor2.setIdleMode(IdleMode.kBrake);
    rightMotor1.setIdleMode(IdleMode.kBrake);
    rightMotor2.setIdleMode(IdleMode.kBrake);

    // Reset Encoder Positions to zero
    leftEncoder.setPosition(0.0);
    rightEncoder.setPosition(0.0);

    maxRPM = 1000;
  }

  @Override
  public void autonomousPeriodic() 
  {
    // ALERT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // Still need to tune Encoder settings for all profiles
    double pos = 0.0;
    double error = 0.0;
    double left = 0.0;
    double right = 0.0;
    

// Switch Statement based on Autonomous Selected
    switch (autoSelectionName)
    {
      // Each Autonomous Case will use another switch statement to
      // handle the stages of the program
      case "Easy":
        switch (stage)
        {
          case 0: // Release the bungee to slap the cube
            slapTheCube();
            Timer.delay(1);
            stage = 1;
            break;

          case 1: // Drive to position "pos"
            pos = -12; // This is in feet...negative values move the robot forward
            leftPID.setReference(pos, CANSparkMax.ControlType.kSmartMotion);
            rightPID.setReference(pos, CANSparkMax.ControlType.kSmartMotion);

            break;

          default:
            break;
        }  
        break;
      
      case "Hard-Balance":
        switch (stage)
        {
          case 0: // Release the bungee to slap the cube
            slapTheCube();
            Timer.delay(1);
            stage = 1;
            break;

          case 1: // Drive to position "pos"
            pos = -12; // This is in feet
            error = pos - leftEncoder.getPosition();

            //leftPID.setFF(0.000656); // Less aggressive acceleration for use with Charge Station
            leftPID.setReference(pos, CANSparkMax.ControlType.kSmartMotion);
            rightPID.setReference(pos, CANSparkMax.ControlType.kSmartMotion);

            stage = Math.abs(error) > 0.1 ? 1 : 2;
            break;

          case 2: // Drive back to Charge Station
            pos = -5; // This is in feet
            error = pos - leftEncoder.getPosition();

            leftPID.setReference(pos, CANSparkMax.ControlType.kSmartMotion);
            rightPID.setReference(pos, CANSparkMax.ControlType.kSmartMotion);

            stage = Math.abs(error) > 0.1 ? 1 : 2;
            if(stage == 2){ Timer.delay(2);}
            break;

          case 3: // balance
            isBalancing = true;
            break;

          default:
            break;
        }
        break;

      case "Hard-Multiple":
        switch (stage)
        {
          case 0: // Release the bungee to slap the cube
            slapTheCube();
            Timer.delay(1);
            stage = 1;
            break;

          case 1: // Drive to position "pos"
            pos = -12; // This is in feet
            error = pos - leftEncoder.getPosition();

            leftPID.setReference(pos, CANSparkMax.ControlType.kSmartMotion);
            rightPID.setReference(pos, CANSparkMax.ControlType.kSmartMotion);

            stage = Math.abs(error) < 0.1 ? 1 : 2;
            break;

          case 2:// Target and acquire Game Piece
            table.getEntry("pipeline").setNumber(CUBE); // Chage Pipeline as needed
            if(Math.abs(txEntry.getDouble(0.0)) > 1.0 && Math.abs(tyEntry.getDouble(0.0)) > 1.0)
            {
              autoAim(txEntry.getDouble(0.0), tyEntry.getDouble(0.0));   
              left = (steering_adjust - distance_adjust) * maxRPM; 
              right = (-steering_adjust - distance_adjust) * maxRPM;         
              leftPID.setReference(left, CANSparkMax.ControlType.kVelocity);
              rightPID.setReference(right, CANSparkMax.ControlType.kVelocity);
            }
            else
            {
              grabberSolenoid.set(false);
              stage = 3;
            }
            break;

          case 3: // Turn toward scoring area and lift leg
            targetYaw = -180;
            isRotating = true;
            left = (steering_adjust) * maxRPM/2; 
            right = (-steering_adjust) * maxRPM/2;         
            leftPID.setReference(left, CANSparkMax.ControlType.kVelocity);
            rightPID.setReference(right, CANSparkMax.ControlType.kVelocity);
            liftLeg();
            Timer.delay(0.1);
            stage = (navX2.getYaw() - 2 > -180) ? 3 : 4;
            
          case 4: // Approach Goal
            // End rotation and hold leg in position
            isRotating = false;
            stopLeg();

            // Reset Encoder positions to zero for ease of use
            leftEncoder.setPosition(0.0);
            rightEncoder.setPosition(0.0);

            // Drive close enough to acquire targeting on AprilTag
            pos = -8; // This is in feet
            error = pos - leftEncoder.getPosition();

            leftPID.setReference(pos, CANSparkMax.ControlType.kSmartMotion);
            rightPID.setReference(pos, CANSparkMax.ControlType.kSmartMotion);

            stage = Math.abs(error) > 0.3 ? 4 : 5;

          case 5: // Target and AutoAim/Approach goal
            // Target and Acquire Cone Scoring pole using AprilTag or Retro Tape
            // MAKE SURE THE SCORING PIPELINE MATCHES THE OBJECT FROM ABOVE
            table.getEntry("pipeline").setNumber(SCORE_CUBE);  

            if(Math.abs(txEntry.getDouble(0.0)) > 1 || 
              Math.abs(tyEntry.getDouble(0.0)) > 1
              )
            {
              autoAim(txEntry.getDouble(0.0), tyEntry.getDouble(0.0));
              left = (steering_adjust - distance_adjust) * maxRPM; 
              right = (-steering_adjust - distance_adjust) * maxRPM;         
              leftPID.setReference(left, CANSparkMax.ControlType.kVelocity);
              rightPID.setReference(right, CANSparkMax.ControlType.kVelocity);            }
            else
            {
              stopLeg();
              grabberSolenoid.set(true);
              stage = 6;
            }

          default:
            break;
        }
        break;

      case "Dream": // Not yet implemented
        break;

      default:
        DriverStation.reportError("Error Selecting Autonomous:  Selected Value is:  " + autoSelectionName, true);
        break;
    }
    
    steering_adjust = 0.0;
    distance_adjust = 0.0;
  }


  // **************  SMARTDASHBOARD CODE  **************************************************************************
  
    public void robotPeriodic() 
  {
    // Check Spotter Joystick for input so SPotter can modify Autonomous Setup
    joySpotter();
    joyBackup();

    // Update Target Yaw to Current Yaw unless the robot is told to rotate
    if(isRotating)
      autoRotateToAngle(targetYaw);

    // Balance the robot
    if(isBalancing)
      balance();

    // Show which Autonomous is Selected
    SmartDashboard.putString("Auto Selected", autoSelectionName);

    // LegLock Limit Switch State
    SmartDashboard.putBoolean("LegLockSwitch", legLocked.get());

    // Toggle for Compressor
    compressorState = SmartDashboard.getBoolean("Compressor", compressorState);

    // I use the negation so that enabled is the fail state for safety purposes
    if(!compressorState)
    {
      myCompressor.disable();
    }
    else
    {
      myCompressor.enableDigital();
    }
    SmartDashboard.putBoolean("Compressor", compressorState);

    // For Testing Purposes
    // NavX Info 
    SmartDashboard.putNumber("NavX Yaw", Math.floor(navX2.getYaw()));
    SmartDashboard.putNumber("NavX Pitch", Math.floor(navX2.getPitch()));
    SmartDashboard.putNumber("NavX Roll", Math.floor(navX2.getRoll()));
    
    // // Limelight Info
    //   //read values periodically
    //   double tx = Math.floor(txEntry.getDouble(0.0));
    //   double ty = Math.floor(tyEntry.getDouble(0.0));
    //   double ta = Math.floor(taEntry.getDouble(0.0));
    //   double tagID = Math.floor(table.getEntry("tid").getDouble(-1.0));

    //   //post to smart dashboard periodically
    //   SmartDashboard.putNumber("LimelightX", tx);
    //   SmartDashboard.putNumber("LimelightY", ty);
    //   SmartDashboard.putNumber("LimelightArea", ta);
    //   SmartDashboard.putNumber("AprilTag Value", tagID);  

  }

  
  // *********************** JOYSTICK/CONTROLLER CODE ***************************************************************

  // The Driver is responsible for robot movement
  // Their joystick will control
  //   Drivetrain, Leg Raising/Lowering, Grabber Open/Close
  //   AutoAim, AutoBalance
  public void joyDriver()
  {
    if(joyDriver.getRawAxis(3) < 0)
    {   needBackup = true;  }
    else if(isRotating)
    {
      // Ignore other inputs and Keep Rotating to last targetYaw
    }
    else if(joyDriver.getRawButton(1))
    {
      // Toggle Grabber
      grabberSolenoid.set(!grabberSolenoid.get());
      Timer.delay(0.25);
    }
    else if(joyDriver.getRawButton(2))
    {
      // Automatically adjust Yaw and Distance to current Limelight pipeline Target
      autoAim(txEntry.getDouble(0.0), tyEntry.getDouble(0.0));
    }
    else if(joyDriver.getRawButton(3))
    {
      unlockLeg();
      liftLeg();
    }
    else if(joyDriver.getRawButton(4))
    {
      lockLeg();
      lowerLeg();
    }
    else if(joyDriver.getRawButton(5))
    {
      // Rotates Directly to the left spinning Counter-Clockwise while holding button
      targetYaw = 90;
      isRotating = true;
    }
    else if(joyDriver.getRawButton(6))
    {
      // Rotates Directly to the right spinning Clockwise while holding button
      targetYaw = -90;
      isRotating = true;
    }
    else
    {
      lockLeg();
      stopLeg();
      isRotating = false;
      needBackup = false;
    }
  }

  // The Spotter is responsible for Autonomous Setup and Target Aquisition
  // Their joystick will control
  //   Autonomous Profile Selection, Pipeline Selection
  public void joySpotter()
  {
    if(joySpotter.getRawButton(1)) // Trigger
    {
      // Re-Calibrate NavX2 Gyro
      navX2.reset();
      Timer.delay(5);
    }
    else if(joySpotter.getRawButton(2)) // Thumb button down
    {
      // Easy Auto Selection
      autoSelectionName = "Easy";
    }
    else if(joySpotter.getRawButton(3)) // Thumb button up
    {
      // Hard-Balance Auto Selection
      autoSelectionName = "Hard-Balance";
    }
    else if(joySpotter.getRawButton(4)) // Thumb button left
    {
      // Hard-Multiple Auto Selection
      autoSelectionName = "Hard-Multiple";
    }
    else if(joySpotter.getRawButton(5)) // Thumb button right
    {
      // Dream Auto Selection
      autoSelectionName = "Dream";
    }
    else if(joySpotter.getRawButton(6)) // Base Button Left Side Top
    {
      // Target Cube on High Loading Area Pipeline
      table.getEntry("pipeline").setNumber(CONE);
    }
    else if(joySpotter.getRawButton(7)) // Base Button Left Side Bottom
    {
      // Target Cube on High Loading Area Pipeline
      table.getEntry("pipeline").setNumber(CUBE);
    }
    else if(joySpotter.getRawButton(8)) // Base Buttons Middle Left
    {
      // AprilTag Crosshair Calibrated for Scoring Cones to LEFT
      table.getEntry("pipeline").setNumber(SCORE_CONE);
    }
    else if(joySpotter.getRawButtonPressed(9)) // Base Buttons Middle Right
    {
      // AprilTag Crosshair Calibrated for Scoring Cones to LEFT
      table.getEntry("pipeline").setNumber(SCORE_CUBE);      
    }
    else if(joySpotter.getRawButtonPressed(10)) // Base Buttons Right side bottom
    {
      // AprilTag Crosshair Calibrated for Scoring Cubes
      table.getEntry("pipeline").setNumber(LOAD_LOW);
    }
    else if(joySpotter.getRawButton(11)) // Base Buttons Bottom Right side top
    {
      // AprilTag Crosshair Calibrated for Scoring Cubes
      table.getEntry("pipeline").setNumber(LOAD_HIGH);
    }
    else // No Assigned Button/Axis
    {
      // Do Nothing
    }
  }
  
  // The Backup Joystick is used if the gyro goes haywire and we need basic Tank Control over the robot
  public void joyBackup()
  {
    if(joyBackup.getRawButton(2)) // A Button on Controller
    {
      // Recalibrate Gyro
      navX2.reset();
    }
  }
  
  
    @Override
  public void robotInit() 
  {

// Initialize Acceleration Curve settings
    // Logistic Curve values
    // acceleration_curvature = -0.1;
    acceleration_midpoint = 100;
    populateArrayList();

    // Reset controller settings and set Idle Mode to Brake
    // Based on advice from Brandon at Cimarron we also limit
    // the current draw of the Neos to prevent brownout
    leftMotor1.restoreFactoryDefaults();
    leftMotor1.setIdleMode(IdleMode.kCoast);
    leftMotor1.setSmartCurrentLimit(80);

    leftMotor2.restoreFactoryDefaults();
    leftMotor2.setIdleMode(IdleMode.kCoast);
    leftMotor2.setSmartCurrentLimit(80);

    rightMotor1.restoreFactoryDefaults();
    rightMotor1.setIdleMode(IdleMode.kCoast);
    rightMotor1.setSmartCurrentLimit(80);

    rightMotor2.restoreFactoryDefaults();
    rightMotor2.setIdleMode(IdleMode.kCoast);
    rightMotor2.setSmartCurrentLimit(80);

    // Set secondary motors on each side as followers
    leftMotor2.follow(leftMotor1);
    rightMotor2.follow(rightMotor1);

    // Set left side motors inverted so that we can use the same 
    // numbers for both sides without worrying about signs
    leftMotor1.setInverted(true);

    // Set Encoder Conversion such that position is in feet (ft) and 
    // velocity is in feet per second (ft/s)
    // Gear Teeth based on our setup of the Toughbox Mini
    double encoderGearTeeth = 14;
    double gear1OuterTeeth = 50;
    double gear1InnerTeeth = 16;
    double gear2OuterTeeth = 48;
    gearRatio = ((encoderGearTeeth/gear1OuterTeeth)*gear1InnerTeeth)/gear2OuterTeeth; // Number of Output shaft revolutions per One Encoder shaft revolution
    wheelDiameter = 0.5; // feet
    encoderRotationsToFeetConversionFactor = gearRatio * Math.PI * wheelDiameter;
    leftEncoder.setPositionConversionFactor(encoderRotationsToFeetConversionFactor);
    rightEncoder.setPositionConversionFactor(encoderRotationsToFeetConversionFactor);

    // Enable the compressor
    myCompressor.enableDigital();

    // Prep Slap Servo
    slapReleaseServo.setAngle(UNLOCK_ANGLE);
    
    // Setup SmartMotion
    leftPID.setP(kP);
    leftPID.setI(kI);
    leftPID.setD(kD);
    leftPID.setIZone(kIz);
    leftPID.setFF(kFF);
    leftPID.setOutputRange(kMinOutput, kMaxOutput);

    rightPID.setP(kP);
    rightPID.setI(kI);
    rightPID.setD(kD);
    rightPID.setIZone(kIz);
    rightPID.setFF(kFF);
    rightPID.setOutputRange(kMinOutput, kMaxOutput);

    int smartMotionSlot = 0;
    leftPID.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    leftPID.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    leftPID.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    leftPID.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    rightPID.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    rightPID.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    rightPID.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    rightPID.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    // Create and calibrate the navX2 sensor package
    try
    {
      navX2 = new AHRS(SerialPort.Port.kUSB);
    }
    catch (RuntimeException ex)
    {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }
    navX2.calibrate();
    navX2.reset();

   
  }

  @Override
  public void teleopInit() 
  {
    isBalancing = false;
    leftMotor1.setIdleMode(IdleMode.kBrake);
    leftMotor2.setIdleMode(IdleMode.kBrake);
    rightMotor1.setIdleMode(IdleMode.kBrake);
    rightMotor2.setIdleMode(IdleMode.kBrake);
  }  
  
  @Override
  public void teleopPeriodic() 
  {
    joyDriver();

    double currentYaw = navX2.getYaw()*Math.PI/180;
    double fowardBackwardAxis = joyDriver.getRawAxis(1) * Math.cos(currentYaw);
    double leftRightAxis = -joyDriver.getRawAxis(0) * Math.sin(currentYaw);
    double twistAxis = -joyDriver.getRawAxis(2);

    // Set deadbands
    fowardBackwardAxis = fowardBackwardAxis < 0.05 && fowardBackwardAxis > -0.05 ? 0 : fowardBackwardAxis;
    leftRightAxis = leftRightAxis < 0.05 && leftRightAxis > -0.05 ? 0 : leftRightAxis;
    twistAxis = twistAxis < 0.1 && twistAxis > -0.1 ? 0 : twistAxis;
  
    double appliedAxis = Math.abs(Math.sin(currentYaw)) >= Math.abs(Math.cos(currentYaw)) ? leftRightAxis : fowardBackwardAxis;
    
    double left = clamp((appliedAxis + twistAxis + steering_adjust - distance_adjust), -1, 1);
    double right = clamp((appliedAxis - twistAxis - steering_adjust - distance_adjust), -1, 1);
  
    // Checking to see if you need Backup
    if(needBackup)
    {
      left = joyBackup.getRawAxis(1);
      right = joyBackup.getRawAxis(3);
    }
    updateMotorSetting(left, right);
    
    steering_adjust = 0.0;
    distance_adjust = 0.0;
  }


  /*
   * Method that takes a Yaw Error (tx) and Roll Error (ty) and sets adjustment variables
   * to bring robot to a position that will reduce those errors to zero
   */
  public void autoAim(double tx, double ty)
  {
    double heading_error = -tx;
    double distance_error = -ty;

    heading_error = heading_error > 30 ? 30 : heading_error;
    heading_error = heading_error < -30 ? -30 : heading_error;

    distance_error = distance_error > 25 ? 25 : distance_error;
    distance_error = distance_error < -25 ? -25 : distance_error;
    
    steering_adjust = heading_error / 60;
    distance_adjust = distance_error / 50;   
  }

  /*
   * Method that takes a desired Yaw angle and uses the AutoAim method
   * to rotate the robot to that desired Yaw Angle
   * 
   * NOTE: The angle should be between -180 and 180 since the NavX2-Micro
   *       returns Yaw values in that range
   */
  public void autoRotateToAngle(double angle)
  {
    double currentYaw = navX2.getYaw();
    autoAim(angle - currentYaw, 0.0);
  }

  // /*
  //  * Method that takes a Yaw Error (tx) and Roll Error (ty) and sets adjustment variables
  //  * to bring robot to a position that will reduce those errors to zero
  //  */
  // public void autoAim(double tx, double ty)
  // {
  //   double KpAim = 0.02;
  //   double KpDistance = 0.02;
  //   double min_aim_command = 0.02;

  //   double heading_error = -tx;
  //   double distance_error = -ty;

  //   heading_error = heading_error > 30 ? 30 : heading_error;
  //   heading_error = heading_error < -30 ? -30 : heading_error;

  //   if (tx > 1.0)
  //   {
  //     steering_adjust = KpAim*heading_error - min_aim_command;
  //   }
  //   else if (tx < -1.0)
  //   {
  //     steering_adjust = KpAim*heading_error + min_aim_command;
  //   }
    
  //   distance_adjust = KpDistance * distance_error;   
  // }

  // /*
  //  * Method that takes a desired Yaw angle and uses the AutoAim method
  //  * to rotate the robot to that desired Yaw Angle
  //  * 
  //  * NOTE: The angle should be between -180 and 180 since the NavX2-Micro
  //  *       returns Yaw values in that range
  //  */
  // public void autoRotateToAngle(double angle)
  // {
  //   double currentYaw = navX2.getYaw();

  //   SmartDashboard.putNumber("CurrentYaw", currentYaw);
  //   SmartDashboard.putNumber("TargetYaw", angle);
    
    
  //   if(currentYaw < angle - 5 || currentYaw > angle + 5)
  //   {
  //     autoAim(angle - currentYaw, 0.0);
  //     //isRotating = true;
  //   }
  //   else
  //   {
  //     //isRotating = false;
  //   }
  // }

  /* 
   * Method that sets the distance adjustment variable based
   * on the Pitch angle from the NavX2-Micro so that it will 
   * roll forward if it is leaning back and vice versa
   */
  public void balance()
  {
    double roll_error = -navX2.getRoll();

    // Set Adjustment for Pitch Correction
    distance_adjust = Math.abs(roll_error) > 10 ? roll_error / 180 : 0;
  }


  /*
   * This method moves the motor through the indices of the
   * ArrayList that holds the pre-calculated motor settings along
   * the Logistics Curve  The setpoint is the desired setpoint
   * and the index should be the current index of the setting for
   * the motor being updated
   */
  public int updateMotorIndex(double setpoint, int index)
  {
    // Full stop error correction for joystick drift
    if(index < maxTicks/2 + 4 && index > maxTicks/2 - 4){
      if(Math.abs(setpoint) < settingsList.get(maxTicks/2 + 1))
      {
        return maxTicks/2;
      }
    }

    double currentSetting = settingsList.get(index);

    // Update Index
    if(currentSetting < setpoint)
    {
      if(index < settingsList.size() - 1)
      {
        index = index + 1;
      }
      else
      {
        index = settingsList.size() - 1;
      }
    }
    else if(currentSetting > setpoint)
    {
      if(index > 0)
      {
        index = index - 1;
      }
      else
      {
        index = 0;
      }
    }

    // Limit speed during in-place rotations and when leg is unlocked
    if(leftMotor1.getAppliedOutput() * rightMotor1.getAppliedOutput() < 0 || !legLocked.get())
    {
      if(index < maxTicks/3)
      {
        index = maxTicks/3;
      }
      else if(index > maxTicks*2/3)
      {
        index = maxTicks*2/3;
      }
    }

    return index;

  }
  /*
   * Use this method to actually move the robot based on the current
   * values of steering_adjust and distance_adjust and Controller axis
   */
  public void updateMotorSetting(double joyModX, double joyModY)
  {
    double left = 0.0;
    double right = 0.0;
    double desired_setpoint_left = steering_adjust + distance_adjust;
    double desired_setpoint_right = -steering_adjust + distance_adjust;

    // Get Desired Setpoints based on Stick 
    desired_setpoint_left += (double)((int)(joyModX*10000))/10000;
    desired_setpoint_right += (double)((int)(joyModY*10000))/10000;


    currentIndexLeft = updateMotorIndex(desired_setpoint_left, currentIndexLeft);
    currentIndexRight = updateMotorIndex(desired_setpoint_right, currentIndexRight);

    left = settingsList.get(currentIndexLeft);
    right = settingsList.get(currentIndexRight);
    
    leftMotor1.set(left);
    rightMotor1.set(right);
    steering_adjust = 0.0;
    distance_adjust = 0.0;
  }

  public void liftLeg()
  {
    valveServo.setAngle(OPEN_ANGLE);
    legSolenoid.set(false);

  }

  public void lowerLeg()
  {
    valveServo.setAngle(OPEN_ANGLE);
    legSolenoid.set(true);

  }

  public void stopLeg()
  {
    valveServo.setAngle(CLOSE_ANGLE);
    legSolenoid.set(false);
  }
  
  public void lockLeg()
  {
    legReleaseServo_1.setAngle(LOCK_ANGLE);
    legReleaseServo_2.setAngle(LOCK_ANGLE);
  }

  public void unlockLeg()
  {
    legReleaseServo_1.setAngle(UNLOCK_ANGLE);
    legReleaseServo_2.setAngle(UNLOCK_ANGLE);
  }

  public void slapTheCube()
  {
    slapReleaseServo.setAngle(LOCK_ANGLE);
  }

  public double getSettingFromCurve(double x)
  {
    return -0.5*Math.cos(Math.PI/acceleration_midpoint*x) + 0.5;
    //return (1/(1+Math.pow(Math.E, acceleration_curvature*(x - acceleration_midpoint))));
  }

  public void populateArrayList()
  {
    /*
     * This is a custom acceleration curve based on the Logistic Function
     * because I couldn't get PID tuning to work with REV kSmartVelocity.
     * This will likely seem like much more work to people who know how PID 
     * tuning works with the SparkMax SmartMotion system but this was easier
     * for me at the time of writing.
     * 
     * The refresh rate for periodic functions is defaulted to 20ms which I 
     * will refer to as 1 "tick" and all functions and the curvature coefficient
     * are based on this unit
     * 
     * The idea is that there are discrete motor settings during both positive and
     * negative acceleration that follow the logistics curve.  These values will be calculated 
     * here and recalculated when changes are made to the curvature and midpoint variables
     * in the Dashboard.  They will be stored in the settingsList ArrayList such that:
     *      settingsList.get(0) = -1 (full reverse)
     *      settingsList.get(size/2) = 0 (full stop)
     *      settingsList.get(size - 1) = 1 (full forward)
     *      at other positions 1 through size-2 will follow the output of the function
     * 
     * The standard Logistics Function as I'm using it is:
     *      y = 1/(1+e^(acceleration_curvature)(x-acceleration_midpoint))
     *      
     *      where y = the motor setting between 0 and 1 which can be modified to be + or - based on direction
     *      and x = the index of the ticks array
     */
    
    settingsList  = new ArrayList<Double>();

    // Populate "empty" List to avoid null pointers
    for(int i = 0; i < maxTicks; i++)
    {
      settingsList.add(0.0);
    }

    // Set values for endpoints
    settingsList.set(0, -1.0);
    settingsList.set(settingsList.size() - 1, 1.0);

    // Set remaining values based on Curve Function
    int k = 1;
    for(int i = settingsList.size() / 2 + 1; i < settingsList.size(); i++)
    {
      settingsList.set(i, getSettingFromCurve(k));
      k++;
    }
    
    for(int i = 1; i < settingsList.size() / 2; i++)
    {
      settingsList.set(i, -getSettingFromCurve(k));
      k--;
    }

    // // Testing output
    // double[] temp = new double[settingsList.size()];
    // for(int i = 0; i < settingsList.size(); i++)
    // {
    //   temp[i] = settingsList.get(i).doubleValue();
    // }
    // SmartDashboard.putNumberArray("settingsArray", temp);
  }
  
  public double clamp(double val, double min, double max) 
  {
    if(val < min) { return min; }
    if(val > max) { return max; }
    return val;
  }


    
  // Motors and Drive Declaration/Insantiation
  private CANSparkMax leftMotor1 = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax leftMotor2 = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax rightMotor1 = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax rightMotor2 = new CANSparkMax(4, MotorType.kBrushless);
  private RelativeEncoder leftEncoder = leftMotor1.getEncoder();
  private RelativeEncoder rightEncoder = rightMotor1.getEncoder();
  private SparkMaxPIDController leftPID = leftMotor1.getPIDController();
  private SparkMaxPIDController rightPID = rightMotor1.getPIDController();

  // Limit Switch(es) Declaration/Instantiation
  private DigitalInput legLocked = new DigitalInput(0);
  
  // Acceleration Curve
  // Two seconds worth of ticks at 20ms per tick is 100, 
  // Double that for positive and negative acceleration
  // Finally, add one for full stop...that's 201 ticks
  // Adjustments will impact the acceleration curve so do 
  // so only if you also refactor the Curve function itself

  private int maxTicks = 201;
  private int currentIndexLeft = maxTicks / 2;
  private int currentIndexRight = maxTicks / 2;
  private ArrayList<Double> settingsList;
  private double acceleration_midpoint;//, acceleration_curvature;
  
  // nav-X Micro
  private AHRS navX2;

  // Joysticks
  private Joystick joyDriver = new Joystick(0);
  private Joystick joySpotter = new Joystick(1);
  private Joystick joyBackup = new Joystick(2); // This will be programmed to supply standard Tank Drive in case the Gyro fails or drifts too far
  private boolean needBackup = false;

  /*  Unused/Available Buttons/Axes on PS Controller
      private final int BACK_BUTTON = 9;
      private final int START_BUTTON = 10;
      private final int LS_BUTTON = 11;
      private final int RS_BUTTON = 12;
      private final int LR_AXIS_LEFT_STICK = 0;
      private final int LR_AXIS_RIGHT_STICK = 2;
  */

  // Pipeline Constants
  private final int CONE = 0; // Retroreflective Threshold for Cones crosshair at bottom to put object in grabber
  private final int CUBE = 1; // Retroreflective Threshold for Cubes crosshair at bottom to put object in grabber
  private final int LOAD_LOW = 2; // AprilTag crosshair centered at height to represent distance for low loading zone
  private final int LOAD_HIGH = 3; // AprilTag crosshair left at height to represent distance for loading to the right side of high loading zone
  private final int SCORE_CUBE = 4; // AprilTag centered at height to represent distance for scoring on middle level cube area
  private final int SCORE_CONE = 5; // Retroreflective Threshold for Retro-Tape on pole with crosshair centered at heigh to represent distance for scoring on middle level cone pole


  // Servo Angle Constants
  private final double OPEN_ANGLE = 120.0;
  private final double CLOSE_ANGLE = 30.0;
  private final double UNLOCK_ANGLE = 0;
  private final double LOCK_ANGLE = 180;

  
  // Robot Hardware
  Compressor myCompressor = new Compressor(PneumaticsModuleType.CTREPCM);
  Solenoid grabberSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 6);
  Solenoid legSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  Servo valveServo = new Servo(0);
  Servo legReleaseServo_1 = new Servo(1);
  Servo legReleaseServo_2 = new Servo(2);
  Servo slapReleaseServo = new Servo(3);
  
  
  // NetworkTables Global Variables for use with Limelight
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry txEntry = table.getEntry("tx");
  NetworkTableEntry tyEntry = table.getEntry("ty");
  NetworkTableEntry taEntry = table.getEntry("ta");

  // Variable that controls autonomous profile selection
  private String autoSelectionName = "Easy";

  // Auto-Aim and Auto-Balance Variables
  private double steering_adjust = 0.0;
  private double distance_adjust = 0.0;

  // Encoder Variables
  private double gearRatio, wheelDiameter, encoderRotationsToFeetConversionFactor;

  // Rotation with navX2 Variables
  private double targetYaw;
  private boolean isRotating = false;
  private boolean isBalancing = false;


  // Compressor Enable/Disable variable
  private boolean compressorState = true;

  // Autonomous Stage Tracking Array
  private int stage = 0;

  // PID Variables for DriveToPosition Method
  // Setup PID for SmartMotion
  private double kP = 5e-5;
  private double kI = 1e-6;
  private double kD = 0.0;
  private double kIz = 1;
  private double kFF = 0.000156; // Aggressive Acceleration...for more moderate acceleration use 0.000156
  private double kMaxOutput = 1;
  private double kMinOutput = -1;
  private double maxRPM = 5700;
  private double maxVel = 5700;
  private double minVel = 0;
  private double maxAcc = 1500;
  private double allowedErr = 0.1; // This is in Feet so our error should be around 1.2 inches +/- but usually -
}





















