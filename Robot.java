
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import java.util.Set;

//falcon 500 
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

//PWM motors
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;


// Network tables
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;



public class Robot extends TimedRobot {
  
  
  double xEntry;
  double yEntry;
  
  //color sensor
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  private final ColorMatch m_colorMatcher = new ColorMatch();

  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);

  //drive motors
  WPI_TalonFX _rghtFront = new WPI_TalonFX(2, "rio");
  WPI_TalonFX _rghtFollower = new WPI_TalonFX(0, "rio");
  WPI_TalonFX _leftFront = new WPI_TalonFX(1, "rio");
  WPI_TalonFX _leftFollower = new WPI_TalonFX(3, "rio");

  //differinal driving class
  DifferentialDrive _diffDrive = new DifferentialDrive(_leftFront, _rghtFront);

  //errors
  Faults _faults_L = new Faults();
  Faults _faults_R = new Faults();

  //intake 
  MotorController m_intake = new PWMSparkMax(2);

  //belt
  MotorController m_belt = new PWMSparkMax(1);

  //elevatorMotor
  MotorController m_elevatorMotor = new PWMSparkMax(0);


  //xbox controllers
  XboxController _xbox = new XboxController(0);

  //compersor
  Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

  //solidnode
  Solenoid intakeSolenoidPCM = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  
  boolean intakeEnabled = false;

  //network tables
  NetworkTableInstance inst = NetworkTableInstance.getDefault();

  NetworkTable table = inst.getTable("limelight");

  //Auto choosable
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  Command pos1;
  Command pos2;


  @Override
  public void robotInit() {
    
    //auto
    
    
    pos1 = new Command() {

      @Override
    public void execute() {
      _diffDrive.arcadeDrive(0.3,0);
    }
      @Override
      public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        return null;
      }
    };

    pos2 = new Command() {

      @Override
    public void execute() {
      _diffDrive.arcadeDrive(-0.3,0);
    }
      @Override
      public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        return null;
      }
    };
    

    m_chooser.setDefaultOption("Simple Auto", pos1);
    m_chooser.addOption("Complex Auto", pos2);

    SmartDashboard.putData(m_chooser);


    intakeEnabled = false;

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
    
    _rghtFront.configFactoryDefault();
    _rghtFollower.configFactoryDefault();
    _leftFront.configFactoryDefault();
    _leftFollower.configFactoryDefault();


    /* set up followers */
    _rghtFollower.follow(_rghtFront);
    _leftFollower.follow(_leftFront);

    /* [3] flip values so robot moves forward when stick-forward/LEDs-green */
    _rghtFront.setInverted(TalonFXInvertType.Clockwise); // !< Update this
    _leftFront.setInverted(TalonFXInvertType.CounterClockwise); // !< Update this

    /*
   * set the invert of the followers to match their respective master controllers
    */
   _rghtFollower.setInverted(InvertType.FollowMaster);
    _leftFollower.setInverted(InvertType.FollowMaster);

    _rghtFront.setSelectedSensorPosition(0);
    _leftFront.setSelectedSensorPosition(0);

  }

  public void deployIntake(){
    m_intake.set(0.05);
    intakeSolenoidPCM.set(false);
  }

  public void retracktIntake(){
    m_intake.set(0);
    intakeSolenoidPCM.set(true);
  }

  public void startElevator(){

  } 

  @Override
  public void teleopInit(){
    intakeEnabled = false;

    _rghtFront.setNeutralMode(NeutralMode.Brake);
    _rghtFollower.setNeutralMode(NeutralMode.Brake);
    _leftFront.setNeutralMode(NeutralMode.Brake);
    _leftFollower.setNeutralMode(NeutralMode.Brake);


  }

  @Override
  public void teleopPeriodic() {

      pcmCompressor.disable();
      // pcmCompressor.enableDigital();

      
      double forw = -_xbox.getRawAxis(1); 
      double turn = -_xbox.getRawAxis(0); 

      // deadband gamepad 10% 
      if (Math.abs(forw) < 0.10) {
          forw = 0;
      }
      else{
        forw = forw *0.7;
      }
      if (Math.abs(turn) < 0.10) {
          turn = 0;
      }else {
          turn = turn * -0.6;
      }

      // arcade drive 
      //_diffDrive.arcadeDrive(forw, turn);

      if(_xbox.getRawButton(1))
      { 


        _diffDrive.arcadeDrive((yEntry+18)*0.04, xEntry*0.03);
      }
      else{
       _diffDrive.arcadeDrive(forw, turn);
      }
      
      //intake motor
      if(_xbox.getRawButtonPressed(5)) intakeEnabled =false;
      if(_xbox.getRawButtonPressed(6)) intakeEnabled =true;
       
      
      if(intakeEnabled){
        deployIntake();

      }else{
        retracktIntake();
        
      }

      m_belt.set(0);
      m_elevatorMotor.set(0);
      
      /* get sensor values */
      // double leftPos = _leftFront.GetSelectedSensorPosition(0);
      // double rghtPos = _rghtFront.GetSelectedSensorPosition(0);
        
    }

  @Override
  public void robotPeriodic() {
    
    xEntry = table.getEntry("tx").getDouble(0);
    yEntry = table.getEntry("ty").getDouble(0);

    SmartDashboard.putNumber("xEntry", xEntry);
    SmartDashboard.putNumber("yEntry", yEntry);

    Color detectedColor = m_colorSensor.getColor();

    String colorString;

    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    SmartDashboard.putString("Detected Color", colorString);

    double leftVelUnitsPer100ms = _leftFront.getSelectedSensorVelocity(0);
    double rghtVelUnitsPer100ms = _rghtFront.getSelectedSensorVelocity(0);

    SmartDashboard.putNumber("Left side speed", leftVelUnitsPer100ms);
    SmartDashboard.putNumber("Right side speed", rghtVelUnitsPer100ms);
    

  }

  @Override
  public void autonomousInit() {
    
  }

  @Override
  public void autonomousPeriodic() {
    m_chooser.getSelected().execute();
  }
}
