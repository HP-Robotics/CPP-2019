/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.CameraServer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public SnazzyMotionPlanner hatchController;
  public TalonPIDOutput hatchPIDOutput;
  public SnazzyMotionPlanner winchController;
  public TalonPIDOutput winchPIDOutput;
  public SnazzyMotionPlanner elevatorController;
  public TalonPIDOutput elevatorPIDOutput;
  public SnazzyMotionPlanner leftController;
  public SnazzyMotionPlanner rightController;
  public SnazzyPIDCalculator spinCalculator;
  public SnazzyMotionPlanner spinnyController;
  public DrivePIDOutput rightPIDOutput;
  public DrivePIDOutput leftPIDOutput;
  public DrivePIDSourceInches leftInInches;
  public DrivePIDSourceInches rightInInches;
  public TurnPIDOutput spinOutput;
  public SpinnyPIDOutput spinnyOutput;
  public LimelightAnglePIDSource spinInput;
  public SpinnyPIDSource spinnyInput;

  private double addToLeft = 0.0;
  private double addToRight = 0.0;

  public static final int DRIVER_STICK1 = 0;
  public static final int DRIVER_STICK2 = 1;
  public static final int OPERATOR_BOX = 2;

  public static final double HATCH_UP = 50.0;//ATLAS 90.0
  public static final double HATCH_DOWN = 107.5;//ATLAS 155.0
  public static final double HATCH_SAFE_BOTTOM = 210.0;
  public static final double HATCH_SAFE_TOP = -1.0;
  public static final double HATCH_EMERGENCY_DOWN = 195.0;

  public static final double ENC_ERROR = 5;
  public static final double HATCH_LEVEL1 = 100;
  public static final double HATCH_LEVEL2 = 4335 -225;
  public static final double HATCH_LEVEL3 = 7810 -225;
  public static final double SAFE_UP = 1900.0;
  public static final double CARGO_LEVEL1 = 2870;
  public static final double CARGO_LEVEL2 = 6432;
  public static final double CARGO_LEVEL3 = 9570;
  public static final double CARGO_CARGO = 4700; 
  public static final double HOP_ELEVATOR = 750;
  public static final double ELEVATOR_ERROR = 100;

  //ANTI-FRANK
  final static double DRIVE_ENC_TO_INCH = Math.PI * 6.0 * (1.0/2048.0);
  final static double DRIVE_INCH_TO_ENC = 1/DRIVE_ENC_TO_INCH;

  //PRO-FRANK
  //final static double DRIVE_ENC_TO_INCH = Math.PI * 6.0 * (24.0/60.0) * (1.0/3.0) * (1.0/256.0)*(156.0/160.0);
	//final static double DRIVE_INCH_TO_ENC = 1/DRIVE_ENC_TO_INCH;
  
  NetworkTable table;
  public int seqNumber = 1;
  public boolean trajStarted = false;
  
  public boolean isUsingIntake;
	

  /* public static final double hatchkA = 0.0000501017;
  public static final double hatchkV = 0.000634177;
  * OLD POT VALUES
  */

  public static final double max_traj_v = 90;
  public static final double max_traj_a = 100;
  public static final double max_traj_j = 300;

  public static final double hatchkA = 0;//0.000501017;
  public static final double hatchkV = 0;//0.00634177;
  public static final double hatchP = 0.02;
  public static final double hatchI = 0.0003;

  public static final double winchP = 0.003;
  public static final double winchI = 0.00001;
  public static final double winch_max_a = 10000;
  public static final double winch_max_v = 4000;

  public static final double elevatorP = 0.00108;
  public static final double elevatorI = 0.0002;
  public static final double elevatorD = 0.02;
  public static final double elevatorkA = 0.0;//0.000005;//0.000095086;
  public static final double elevatorkV = 0;//0.00183371;
  public final static double elevator_max_a = 8000;
  public final static double elevator_max_v = 5555;

  public boolean firstHop = false;
  public boolean hopping = false;
   public double hoptime;

  public static final double spinP = 0.05;
  public static final double spinI = 0.00001;
  public static final double spinD = 0.2;

  //FRANK
  /*final double driveP = 0.3+0.4;
	final double driveI = 0.005+0.01;
	final double driveD = 1.0;
	final double drivekV = 0.00246*1.15;
  final double drivekA = 0.0108;
  final double drivetkA = 0.044;
  final double drivetkV = 0.178;*/
  



  //CALYPSO + ATLAS
  public final static double driveP = 0.5*.75;
  public final static double driveI = 0.025;
  public final static double driveD = 0.2;
  public static final double drivekV = 0.0075407;
  public static final double drivekA = 0.00285438;
  final double drivetkA = 0.000352641;
  final double drivetkV = 0.00143277;
  
  // 38.1 is about 180 degrees, 18.4 is about 90
  public boolean hatchDown = true;
  public boolean calibrating = false;
  public boolean pidTuning = false;

  //ATLAS
  public TalonSRX topRight;
  public TalonSRX topLeft;
  public TalonSRX bottomRight;
  public TalonSRX bottomLeft;

  //CALYPSO
  /*public VictorSPX topRight;
  public VictorSPX topLeft;
  public VictorSPX bottomRight;
  public VictorSPX bottomLeft;*/


  public Encoder driveLeftEnc;
  public Encoder driveRightEnc;
  public Encoder elevatorEnc;
  public Encoder winchEnc;

  public Potentiometer winchPot;
  public Potentiometer hatchPot;

  public Joystick driverStick1;
  public Joystick driverStick2;
  public Joystick operatorBox;

  public TalonSRX roller; //ATLAS
  //public VictorSPX roller; //CALYPSO
  public TalonSRX leftSDS;
  public TalonSRX rightSDS; //ATLAS
  //public VictorSPX rightSDS; //CALYPSO

  public TalonSRX hatch;
  public TalonSRX elevator; //ATLAS
  public TalonSRX winch;

  public DigitalInput winchDown;  //ATLAS

  public Button thumb1;
  public Button trigger1;
  public Button aButton1;
  public Button bButton1;
  public Button xButton1;
  public Button yButton1;
  public Button rocketLeftButton;
  public Button thumb2;
  public Button trigger2;
  public Button aButton2;
  public Button bButton2;
  public Button xButton2;
  public Button yButton2;
  public Button hatchInButton1;
  public Button hatchOutButton1;
  public Button hatchInButton2;
  public Button hatchOutButton2;
  public Button calibrateButton;
  public Button winchToggleButton;
  public Button elevatorHopButton;
  public Button stepAutoButton;
  /*public Button rocketLeftFarButton;
  public Button rocketRightFarButton;
  public Button rocketRightButton;
  public Button lRocketFeederButton;*/



  public Button resetButton; //ELEVATOR HOP
  public Button hatch1;
  public Button hatch2;
  public Button hatch3;
  public Button cargo1;
  public Button cargo2;
  public Button cargo3;
  public Button hatchFeeder;
  public Button hatchToggle;
  public Button sdsIn;
  public Button sdsOut;
  public Button shipHatch;
  public Button shipCargo;
 
  public AxisButton magicButton;
  public AxisButton sdsOperator;
  public double sdsState = 0;

  public ButtonGrouper elevatorButtons;
 
  public double lVertAngle;
  public double rVertAngle;
  public double horzAngle;
  public double dx;
  public double mdy;
  public double ody;
  public double ldy;
  public double rdy;
  public double xp;
  public double yp;
  public double heading;

  public LiteButton lb;
  public Button[] elevatorButtonArray;

  // TODO - Add trajectories for sandstorm moves
  /*double[][] racetrackTurnPlan = {{0, 0, 0},  {48, -48, -90}, {0, -96, -180}};
  double[][] leftRocketClosePlan = {{0, 0, 0},{30,0,0},{127.5-10.5, 95.7-(23.0/4.0), 28.75}};
  double[][] rightRocketClosePlan = {{0, 0, 0},{30,0,0},{127.5-10.5, -(95.7-(23.0/4.0)), -28.75}};
  double[][] stepAutoPlan = {{0,0,0},{50,0,0}};
  double[][] leftRocketFarAutoPlan = {{0,0,0},{36,0,0},{200,50,0}};
  double[][] rightRocketFarAutoPlan = {{0,0,0},{36,0,0},{200,-50,0}};
  double[][] leftCloseRocketBackPlan = {{0,0,0},{48, -48,-61.25}}; //TODO //TODO 
  double[][] leftCloseRocketFeederPlan = {{0,0,0},{48, 60, 90},{48,150,90}}; //TODO //TODO */
  

  public double[] winchArray = {0, /*923,*/ 2592};
  public int winchPos = 0;
  public int winchCount = 0;
  public boolean winchDefault = true;
  
  /*TrajectoryPlanner racetrackTurnTraj;
  TrajectoryPlanner leftRocketCloseTraj;
  TrajectoryPlanner rocketLeftFarTraj;
  TrajectoryPlanner rocketRightFarTraj;
  TrajectoryPlanner rightRocketCloseTraj;
  TrajectoryPlanner leftCloseToShipTraj;
  TrajectoryPlanner leftShipToFeederTraj;

  TrajectoryPlanner stepTraj;*/

  StepAuto steppy;
  public boolean steppyActive = false;

  public boolean spinnyActive = false;

  /*LeftRocketFarAuto leftFarAuto;
  public boolean leftFarActive = false;

  RightRocketFarAuto rightFarAuto;
  public boolean rightFarActive = false;

  LeftRocketCloseAuto leftCloseAuto;
  public boolean rocketLeftCloseActive = false;

  RightRocketCloseAuto rightCloseAuto;
  public boolean rocketRightCloseActive = false;

  LeftRocketToFeederAuto lRocketFeederAuto;
  public boolean lRocketFeederActive = false;*/

  // PRO FRANK ONLY
	//DoubleSolenoid driveSolenoid;
  //Compressor compressor;
  //DoubleSolenoid.Value lowGear = DoubleSolenoid.Value.kForward;


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

    /*CameraServer camera = CameraServer.getInstance();
		if(camera != null) {
			System.out.println("A camera was found");
			UsbCamera c = camera.startAutomaticCapture();
			if(c != null) {
        System.out.println("And it started.");
        //c.setVideoMode(PixelFormat.kMJPEG,320, 240, 15);
			  c.setResolution(320, 240);
				c.setFPS(15);  
			}
    }*/
    
    /*racetrackTurnTraj = new TrajectoryPlanner(racetrackTurnPlan,50, 50, 50, "RacetrackTurn");
    racetrackTurnTraj.generate();
    

    rocketLeftFarTraj = new TrajectoryPlanner(leftRocketFarAutoPlan, max_traj_v, max_traj_a, max_traj_j, "RocketLeftFar");
    rocketLeftFarTraj.generate();

    rocketRightFarTraj = new TrajectoryPlanner(rightRocketFarAutoPlan, max_traj_v, max_traj_a, max_traj_j, "RocketRightFar");
    rocketRightFarTraj.generate();


    rightRocketCloseTraj = new TrajectoryPlanner(rightRocketClosePlan, max_traj_v*0.5, max_traj_a*0.5, max_traj_j*0.5, "RocketRight");
    rightRocketCloseTraj.generate();

    leftRocketCloseTraj = new TrajectoryPlanner(leftRocketClosePlan, max_traj_v*0.5, max_traj_a*0.5, max_traj_j*0.5, "RocketLeft");
    leftRocketCloseTraj.generate();

    leftCloseToShipTraj = new TrajectoryPlanner(leftCloseRocketBackPlan,max_traj_v*0.6, max_traj_a*0.6, max_traj_j*0.6,"Left Back");
    leftCloseToShipTraj.generate();

    leftShipToFeederTraj = new TrajectoryPlanner(leftCloseRocketFeederPlan,max_traj_v*0.7, max_traj_a*0.7, max_traj_j*0.7,"Left Feeder");
    leftShipToFeederTraj.generate();

    stepTraj = new TrajectoryPlanner(stepAutoPlan, max_traj_v*0.7, max_traj_a*0.7, max_traj_j*0.7, "StepAuto");
    stepTraj.generate();*/

    steppy = new StepAuto(this);

    /*leftFarAuto = new LeftRocketFarAuto(this);

    rightFarAuto = new RightRocketFarAuto(this);

    leftCloseAuto = new LeftRocketCloseAuto(this);
    rightCloseAuto = new RightRocketCloseAuto(this);

    lRocketFeederAuto = new LeftRocketToFeederAuto(this);*/

    driverStick1 = new Joystick(DRIVER_STICK1);
    driverStick2 = new Joystick(DRIVER_STICK2);
    operatorBox = new Joystick(OPERATOR_BOX);

    /**
     * CAN IDs
     * 
     * <9: Don't use
     * 10-19: Drive train
     * 20: PDP don't use
     * 21-29: Misc
     * 30-39: Intake stuff
     * 40-49: Elevator stuff
     * 
     * Feel free to change
     */

    aButton1 = new Button(driverStick1, 5, "A");
    bButton1 = new Button(driverStick1, 6, "B");
    xButton1 = new Button(driverStick1, 3, "X");
    yButton1 = new Button(driverStick1, 4, "Y");
    trigger1 = new Button(driverStick1, 1, "SDS Out");
    rocketLeftButton = new Button(driverStick2, 8, "DATA EXPUNGED");
    /*lRocketFeederButton = new Button(driverStick2, 9, "DATA EXPUNGED");
    rocketRightButton = new Button(driverStick2, 14, "DATA EXPUNGED");
    rocketLeftFarButton = new Button(driverStick2, 7, "DATA EXPUNGED");
    rocketRightFarButton = new Button(driverStick2, 13, "DATA EXPUNGED");*/

    //thumb1 = new Button(driverStick1, 2, "SDS Out");

    aButton2 = new Button(driverStick2, 5, "A");
    bButton2 = new Button(driverStick2, 6, "B");
    xButton2 = new Button(driverStick2, 3, "X");
    yButton2 = new Button(driverStick2, 4, "Y");
    trigger2 = new Button(driverStick2, 1, "SDS In");
    thumb2 = new Button(driverStick2, 2, "SDS Out");
    stepAutoButton = new Button(driverStick2, 10, "Step");

    calibrateButton = new Button(driverStick1, 5, "Lib Owned");
    elevatorHopButton = new Button(driverStick2, 5, "Hop");

    winchToggleButton = new Button(driverStick1, 2, "Winch Toggle");
    isUsingIntake = false;

    hatchInButton2 = new Button(driverStick2, 4, "Hatch In");

    resetButton = new Button(operatorBox, 4, "Reset Button");
    shipCargo = new Button(operatorBox, 11, "Cargo Ship Cargo");
    shipHatch = new Button(operatorBox, 5, "Cargo Ship Hatch");
    cargo3 = new Button(operatorBox, 12, "Cargo Level 3");
    hatch3 = new Button(operatorBox, 10, "Hatch Level 3");
    cargo2 = new Button(operatorBox, 6, "Cargo Level 2");
    hatch2 = new Button(operatorBox, 9, "Hatch Level 2");
    cargo1 = new Button(operatorBox, 3, "Cargo Level 1");
    hatch1 = new Button(operatorBox, 8, "Hatch Level 1");
    hatchFeeder = new Button(operatorBox, 2, "Hatch Feeder");
    hatchToggle = new Button(operatorBox, 1, "Hatch Toggle"); // change key
    //sdsIn and sdsOut are actually joysticks, so is magic button
    sdsOperator = new AxisButton(operatorBox, 0, "SDS Switch");
    magicButton = new AxisButton(operatorBox, 1, "Magic Button");


    lb = new LiteButton();
    Button[] elevatorButtonArray =  {cargo3, cargo2, cargo1, hatch3, hatch2, hatch1, shipCargo, shipHatch};
    elevatorButtons = new ButtonGrouper(elevatorButtonArray, lb);

    //ATLAS
    winchDown = new DigitalInput(9);
    
    topLeft = new TalonSRX(10);
    bottomLeft = new TalonSRX(11);
    topRight = new TalonSRX(12);
    bottomRight = new TalonSRX(13);

    //CALYPSO
    /*topLeft = new VictorSPX(10);
    bottomLeft = new VictorSPX(11);
    topRight = new VictorSPX(12);
    bottomRight = new VictorSPX(13);*/

    //PRO-FRANK
    /*topLeft = new TalonSRX(1);
    topLeft.setInverted(true);
    bottomLeft = new TalonSRX(2);
    bottomLeft.setInverted(true);
    topRight = new TalonSRX(3);
    topRight.setInverted(true);
    bottomRight = new TalonSRX(4);
    bottomRight.setInverted(true);*/

    //ATLAS
    driveRightEnc = new Encoder(11, 10, false, EncodingType.k4X);
    driveLeftEnc = new Encoder(13, 12, true, EncodingType.k4X);

    /*CALYPSO ONLY, FLIP THE RIGHT ONE ON ATLAS REEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE */
    /*driveLeftEnc = new Encoder(10, 11, false, EncodingType.k4X);
    driveRightEnc = new Encoder(13, 12, true, EncodingType.k4X);*/

    elevatorEnc = new Encoder(23, 24, true, EncodingType.k4X); //ATLAS
    winchEnc = new Encoder(21,22,false, EncodingType.k4X); //ATLAS
    //winchEnc = new Encoder(0,1,false, EncodingType.k4X);  // CALYPSO


    //PRO-FRANK
    //driveLeftEnc = new Encoder(2, 3, false, Encoder.EncodingType.k4X);
		//driveLeftEnc.setDistancePerPulse(1);
		//driveRightEnc = new Encoder(0, 1, true, Encoder.EncodingType.k4X);
		//driveRightEnc.setDistancePerPulse(1);

    //winch = new AnalogPotentiometer(0, 360, 30);
    hatchPot = new AnalogPotentiometer(4, 270, 0); /* 2700 Max, 2610 Min */ // 4 ATLAS; 0 CALYPSO
    //hatchPot = new AnalogPotentiometer(0, 270, 0); /* 2700 Max, 2610 Min */ // 4 ATLAS; 0 CALYPSO

    //AnalogInput ai1 = new AnalogInput(0);
    //AnalogInput ai2 = new AnalogInput(2);

    //winch = new AnalogPotentiometer(ai1, 360, 30);
    //hatchPot = new AnalogPotentiometer(ai2, 360, 30);

    //ATLAS
    
    roller = new TalonSRX(30);
    leftSDS = new TalonSRX(40);
    rightSDS = new TalonSRX(22);
    
    //CALYPSO
    /*roller = new VictorSPX(30);
    leftSDS = new TalonSRX(40);
    rightSDS = new VictorSPX(22);*/

    //ANTI-FRANK
    winch = new TalonSRX(3);  //ATLAS was 3
    //PRO-FRANK
    //winch = new TalonSRX(59);
    hatch = new TalonSRX(31);
    elevator = new TalonSRX(21); //ATLAS
    //elevator = new VictorSPX(0);


    SmartDashboard.putNumber("P", 0.0);
    SmartDashboard.putNumber("I", 0.0);
    SmartDashboard.putNumber("D", 0.0);
    SmartDashboard.putNumber("Setpoint", 0.0);

    SmartDashboard.putNumber("Trajectory Request", 0);

    SmartDashboard.putBoolean("Elevator Hop", true);

    hatchPIDOutput = new TalonPIDOutput(hatch, -1.0);
    winchPIDOutput = new TalonPIDOutput(winch, 1.0);
    elevatorPIDOutput = new TalonPIDOutput(elevator, -1.0); //ATLAS
    rightPIDOutput = new DrivePIDOutput(topRight, bottomRight, -1.0); 
    leftPIDOutput = new DrivePIDOutput(topLeft, bottomLeft, 1.0);

    leftInInches = new DrivePIDSourceInches(driveLeftEnc);
    rightInInches = new DrivePIDSourceInches(driveRightEnc);

    spinOutput = new TurnPIDOutput();
    spinInput = new LimelightAnglePIDSource();

    spinnyOutput = new SpinnyPIDOutput(topLeft, bottomLeft, topRight, bottomRight, 1.0);
    spinnyInput = new SpinnyPIDSource(driveLeftEnc, driveRightEnc);

    hatchController = new SnazzyMotionPlanner(hatchP, hatchI, 0, 0, hatchkA, hatchkV, 0, 0, hatchPot, hatchPIDOutput, 0.01, "hatch.csv", this);
    winchController = new SnazzyMotionPlanner(winchP, winchI, 0, 0, 0, 0, 0, 0, winchEnc, winchPIDOutput, 0.01, "winch.csv", this);
    elevatorController = new SnazzyMotionPlanner(elevatorP, elevatorI, elevatorD, 0, elevatorkA, elevatorkV, 0, 0, elevatorEnc, elevatorPIDOutput, 0.01, "elevator.csv", this);
    elevatorController.setOutputRange(-0.4, 1.0);
    elevatorController.setProtect(-0.1, 0.2, 100); //ATLAS
    //leftController = new SnazzyMotionPlanner(driveP, driveI, driveD, 0, drivekA, drivekV, -drivetkA, -drivetkV, leftInInches, leftPIDOutput, 0.015, "left.csv", this);
    //rightController = new SnazzyMotionPlanner(driveP, driveI, driveD, 0, drivekA, drivekV, drivetkA, drivetkV, rightInInches, rightPIDOutput, 0.015, "right.csv", this);

    spinCalculator = new SnazzyPIDCalculator(spinP, spinI, spinD, 0.0, spinInput, spinOutput, 0.02, "spin.csv");
    spinCalculator.setOutputRange(-0.5, 0.5);
    spinnyController = new SnazzyMotionPlanner(spinP, spinI, spinD, 0.0,0.0, 0.0, 0.0, 0.0, spinnyInput, spinnyOutput, 0.015, "spinny.csv", this);
    // PRO FRANK ONLY
    //driveSolenoid = new DoubleSolenoid(2, 3);
		//compressor = new Compressor(0);
    

    /* GREMLIN Mitigation 
        It seems to appease the Gremlin, if we let the pid loop run for a while.  We can do that
        while the robot is running, because motor outputs are disabled.  Not ideal, but... */
    winchController.configureGoal(0, winch_max_a, winch_max_v, true);
    winchController.enable();

    hatchController.configureGoal(0, 500, 500, true);
    hatchController.enable();

    /*leftController.configureGoal(0, 10, 10, false);
    leftController.enable();
    rightController.configureGoal(0, 10, 10, false);
    rightController.enable();*/

    elevatorController.configureGoal(0, elevator_max_a, elevator_max_v, false); // ATLAS
    elevatorController.enable(); // ATLAS

  }
  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    //ATLAS
    //TODO - Smartdashboard?  Button?
    teleopInit();
    /*if (SmartDashboard.getBoolean("Elevator Hop", true)) {
      elevatorController.setOutputRange(0.001, 0.002);
      elevatorController.configureGoal(/*HOP_ELEVATOR 100, elevator_max_v, elevator_max_a, true);  //ATLAS
      hopping = true;
      hoptime = Timer.getFPGATimestamp();
  }*/
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
  }
  @Override
  public void teleopInit() {

    /* Gremlin mitigation - we don't want the drive train running while we roll... */
    //leftController.reset();
    //rightController.reset();

    //elevatorController.configureGoal(0, elevator_max_v, elevator_max_a, true);
    //hatch.set(ControlMode.PercentOutput, 0.0);
    //winch.set(ControlMode.PercentOutput, 0.0);
    //elevator.set(ControlMode.PercentOutput, 0.0);
    //leftSDS.set(ControlMode.PercentOutput, 0.0);
    //rightSDS.set(ControlMode.PercentOutput, 0.0);
    winchController.configureGoal(0, winch_max_v, winch_max_a, true);
    winchController.enable();
    
    hatchController.setSetpoint(hatchPot.get());  /* Gremlin mitigation.  If the gremlin prevents this from working, at least hold current position */
    //hatchController.configureGoal(0, 500, 500, true);
    hatchController.enable();

    // PRO FRANK
    //driveSolenoid.set(lowGear);
  }
    
  /** 
   * This function is called periodically during operator control.
   * 
   */
  @Override
  public void teleopPeriodic() {
    /*if(!firstHop && hopping&& (Timer.getFPGATimestamp()-hoptime)>=1.0){
      elevatorController.setOutputRange(-0.4, 1.0);
      elevatorController.configureGoal(HOP_ELEVATOR-elevatorEnc.get(), elevator_max_v, elevator_max_a, true);
      firstHop = true;
    }
    else if(hopping&&((elevatorEnc.get()>= HOP_ELEVATOR-ELEVATOR_ERROR)||(Timer.getFPGATimestamp()-hoptime)>3.0)){
      elevatorController.configureGoal(0-elevatorEnc.get(), elevator_max_v, elevator_max_a, false);
      hopping = false;
    }*/
    dashboardPuts();
    updateButtons();
    if(calibrating) {
      calibrateNow(winchController);
      return;
    }
    if(pidTuning) {
      pidTuneNow(winchController);
      return;
    }
    hatchLogic();
    winchLogic();
    intakeLogic();
    autoDriveLogic();
    magicLogic();
    if(!trajStarted && /*!rocketLeftCloseActive && */!steppyActive /*&& !leftFarActive && !rightFarActive && !rocketRightCloseActive*/&& !spinnyActive/* && !lRocketFeederActive*/){
      drivingLogic();
    }

    elevatorLogic(); //ATLAS
    
  }

  @Override
  public void disabledInit(){
    hatchController.disable();
    winchController.disable();
    elevatorController.disable(); //ATLAS
    //leftController.disable();
    //rightController.disable();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public void updateButtons(){
    aButton1.update();
    bButton1.update();
    xButton1.update();
    yButton1.update();
    trigger1.update();
    //thumb1.update();
    aButton2.update();
    bButton2.update();
    rocketLeftButton.update();
    //rocketRightButton.update();
    xButton2.update();
    yButton2.update();
    trigger2.update();
    thumb2.update();
    stepAutoButton.update();
    //rocketLeftFarButton.update();
    //rocketRightFarButton.update();
    hatchInButton2.update();
    hatchToggle.update();
    calibrateButton.update();
    winchToggleButton.update();
    elevatorButtons.update();
    sdsOperator.update();
    magicButton.update();
    elevatorHopButton.update();
    resetButton.update();
    hatchFeeder.update();
    //lRocketFeederButton.update();

  }
  public void intakeLogic(){

    //System.out.println(sdsOperator.getState());


    if(trigger2.on()||sdsOperator.getState() == -1.0){
      leftSDS.set(ControlMode.PercentOutput, -0.5);
      rightSDS.set(ControlMode.PercentOutput, 0.5);
      //roller.set(ControlMode.PercentOutput, -0.50); //ATLAS
      if(!elevatorAtCargo()){
        roller.set(ControlMode.PercentOutput, -0.45);
       } //Helps Intake on Calypso, try on Atlas
      trigger1.toggleOff();
      System.out.println("in");
      lb.light(trigger2);
      lb.unlight(trigger1);
    }
    if(trigger1.on()||sdsOperator.getState()==1.0){
      isUsingIntake = true;
      leftSDS.set(ControlMode.PercentOutput, 1.0);
      rightSDS.set(ControlMode.PercentOutput, -1.0);
      if(!elevatorAtCargo()){
        roller.set(ControlMode.PercentOutput, 0.15);
      }
      trigger2.toggleOff();
      System.out.println("out");
      lb.light(trigger1);
      lb.unlight(trigger2);
    }

    if(!trigger1.on()&&!trigger2.on() && sdsOperator.getState()==0.0){
      leftSDS.set(ControlMode.PercentOutput, 0.0);
      rightSDS.set(ControlMode.PercentOutput, 0.0);
      roller.set(ControlMode.PercentOutput, 0.0);
      //System.out.println("TURN OFFFFF");
      //lb.unlight(thumb1);
      lb.unlight(trigger1);
      lb.unlight(trigger2);
      isUsingIntake = false;
    }
    /*if(!isUsingIntake && winchToggleButton.changed()){
      if(winchController.getSetpoint() == WINCH_UP_SETPOINT){
        winchController.configureGoal(WINCH_DOWN_SETPOINT, 100, 100, true);
      }else{
        winchController.configureGoal(WINCH_UP_SETPOINT, 100, 100, true);
      }
    }*/
  }

  public void autoDriveLogic() {
    /*if(rocketLeftButton.held()) {
      if(rocketLeftButton.changed()) {
        driveRightEnc.reset();
        driveLeftEnc.reset();
        leftCloseAuto.init();
        rocketLeftCloseActive = true;
      }else{
        leftCloseAuto.periodic();
      }
    } else {
      if(rocketLeftCloseActive){
        rightController.disable();
        leftController.disable();
        leftCloseAuto.stopAll();
        leftCloseAuto.nextStage();
        rocketLeftCloseActive = false;
      }
    }*/

    if(xButton1.held()) {
      if(xButton1.changed()) {
        driveRightEnc.reset();
        driveLeftEnc.reset();
        spinnyController.configureGoal(70.0, 180, 300);
        spinnyController.enable();
        spinnyActive = true;
      }else{
        //leftCloseAuto.periodic();
      }
    } else {
      if(spinnyActive){
        spinnyController.disable();
        spinnyActive = false;
      }
    }

    /*if(rocketRightButton.held()) {
      if(rocketRightButton.changed()) {
        driveRightEnc.reset();
        driveLeftEnc.reset();
        rightCloseAuto.init();
        rocketRightCloseActive = true;
      }else{
        rightCloseAuto.periodic();
      }
    } else {
      if(rocketRightCloseActive){
        rightController.disable();
        leftController.disable();
        rightCloseAuto.stopAll();
        rightCloseAuto.nextStage();
        rocketRightCloseActive = false;
      }
    }*/

    // if(stepAutoButton.held()){
    //   if(stepAutoButton.changed()){
    //     steppy.init();
    //     steppyActive = true;
    //   }else{
    //     steppy.periodic();
    //   }
    // }else{
    //   if(steppyActive){
    //     rightController.disable();
    //     leftController.disable();
    //     steppy.stopAll();
    //     steppy.nextStage();
    //     steppyActive = false;
    //   }
      
    // }

    /*if(rocketLeftFarButton.held()){
      if(rocketLeftFarButton.changed()){
        leftFarAuto.init();
        leftFarActive = true;
      }else{
        leftFarAuto.periodic();
      }
    }else{
      if(leftFarActive){
        rightController.disable();
        leftController.disable();
        leftFarAuto.stopAll();
        leftFarAuto.nextStage();
        leftFarActive = false;
      }
      
    }

    if(rocketRightFarButton.held()){
      if(rocketRightFarButton.changed()){
        rightFarAuto.init();
        rightFarActive = true;
      }else{
        rightFarAuto.periodic();
      }
    }else{
      if(rightFarActive){
        rightController.disable();
        leftController.disable();
        rightFarAuto.stopAll();
        rightFarAuto.nextStage();
        rightFarActive = false;
      }
      
    }

    if(lRocketFeederButton.held()){
      if(lRocketFeederButton.changed()){
        lRocketFeederAuto.init();
        lRocketFeederActive = true;
      }else{
        lRocketFeederAuto.periodic();
      }
    }else{
      if(lRocketFeederActive){
        rightController.disable();
        leftController.disable();
        lRocketFeederAuto.stopAll();
        lRocketFeederAuto.nextStage();
        lRocketFeederActive = false;
      }
      
    }
    */
  }

  public void eightMagicLogic(){
    if(magicButton.held()) {
      if(magicButton.changed()){
        seqNumber++;
        SmartDashboard.putNumber("Trajectory Request", seqNumber);
        trajStarted = false;
      } else{
          if(!trajStarted){
            if(SmartDashboard.getNumber("Trajectory Response", 0) == seqNumber){
              Trajectory t = deserializeTraj();
              TrajectoryPlanner tp = new TrajectoryPlanner(t, "Magic");
              tp.regenerate();
              driveRightEnc.reset();
              driveLeftEnc.reset();
              leftController.configureTrajectory(tp.getLeftTrajectory(), false);
              rightController.configureTrajectory(tp.getRightTrajectory(), false);
              leftController.enable();
              rightController.enable();
              trajStarted = true;
            }
          }
      }
        
      // TODO Magic code
    } else {
        if(trajStarted){
          rightController.disable();
          leftController.disable();
          trajStarted = false;
        }
      // TODO Disable Magic Code
    }
  }

  public void magicLogic() {
    addToLeft = 0.0;
    addToRight = 0.0;
    if(!magicButton.held()) {
      if(spinCalculator.isEnabled()) {
        spinCalculator.disable();
      }
      return;
    } 
    if(!spinInput.isValid()) {
      return;
    }
    if(!spinCalculator.isEnabled()) {
      spinCalculator.enable();
    }
    //spinCalculator.setPID(SmartDashboard.getNumber("P", 0.0), SmartDashboard.getNumber("I", 0.0), SmartDashboard.getNumber("D", 0.0));
    spinCalculator.calculate();
    spinCalculator.m_pidOutput.pidWrite(spinCalculator.m_result);
    addToLeft = -spinOutput.m_value;
    addToRight = -spinOutput.m_value;
  }

  public void drivingLogic(){
    // TODO - hold a button to allow drive outputs to drive together
    if (thumb2.held()) {
      topLeft.set(ControlMode.PercentOutput, -driverStick2.getRawAxis(1) + addToLeft);
      bottomLeft.set(ControlMode.PercentOutput, -driverStick2.getRawAxis(1) + addToLeft);
      topRight.set(ControlMode.PercentOutput, driverStick2.getRawAxis(1) + addToRight);
      bottomRight.set(ControlMode.PercentOutput, driverStick2.getRawAxis(1) + addToRight);
    } else {
      topLeft.set(ControlMode.PercentOutput, -driverStick1.getRawAxis(1) + addToLeft);
      bottomLeft.set(ControlMode.PercentOutput, -driverStick1.getRawAxis(1) + addToLeft);
      topRight.set(ControlMode.PercentOutput, driverStick2.getRawAxis(1) + addToRight);
      bottomRight.set(ControlMode.PercentOutput, driverStick2.getRawAxis(1) + addToRight);
    }
  }

  public void elevatorLogic(){
    if (!elevatorController.isEnabled()){
      elevatorController.enable();
    }


    //TODO - Allow elevator to move among high levels even if the winch is not down
    // TODO - allow a little below level1 too
    if(winchDown.get()|| elevatorEnc.get()>(SAFE_UP-ELEVATOR_ERROR)){ //COMMENT IN FOR ATLAS
      if((hatch1.changed()&&hatch1.on())|| (shipHatch.changed()&&shipHatch.on())|| (hatchFeeder.changed()&&hatchFeeder.on()) )
      {
        if(winchDown.get()){
        elevatorController.configureGoal(HATCH_LEVEL1-elevatorEnc.get(), elevator_max_v, elevator_max_a, false);
        }
      } 
      else if(cargo1.changed()&&cargo1.on() )
      {
        elevatorController.configureGoal(CARGO_LEVEL1-elevatorEnc.get(), elevator_max_v, elevator_max_a, true);
      } 
      else if(hatch2.changed()&&hatch2.on())
      {
        elevatorController.configureGoal(HATCH_LEVEL2-elevatorEnc.get(), elevator_max_v, elevator_max_a,true);
      }
      else if(cargo2.changed()&&cargo2.on())
      {
        elevatorController.configureGoal(CARGO_LEVEL2-elevatorEnc.get(), elevator_max_v, elevator_max_a, true);
      }
      else if(hatch3.changed()&&hatch3.on())
      {
        elevatorController.configureGoal(HATCH_LEVEL3-elevatorEnc.get(), elevator_max_v, elevator_max_a,true);
      }
      else if(cargo3.changed()&&cargo3.on())
      {
        elevatorController.configureGoal(CARGO_LEVEL3-elevatorEnc.get(), elevator_max_v, elevator_max_a,true);
      }
      else if(shipCargo.changed()&&shipCargo.on())
      {
        elevatorController.configureGoal(CARGO_CARGO-elevatorEnc.get(), elevator_max_v, elevator_max_a,true);
      }
      
    }
    if(elevatorEnc.get() < (CARGO_LEVEL1) && (resetButton.changed()|| elevatorHopButton.changed()))
      {
        elevatorController.configureGoal(HOP_ELEVATOR, elevator_max_v, elevator_max_a, false);
      }
    
  }

public boolean elevatorAtCargo(){
  if(isClose(elevatorController.getSetpoint(), CARGO_CARGO, 5) || isClose(elevatorController.getSetpoint(), CARGO_LEVEL1, 5) || isClose(elevatorController.getSetpoint(), CARGO_LEVEL2, 5) || isClose(elevatorController.getSetpoint(), CARGO_LEVEL3, 5)){
    return true;
  } else{
    return false;
  }
}

  public void winchLogic(){
    if (!winchController.isEnabled()){
      winchController.enable();
    }
    if(winchToggleButton.changed()){
      winchDefault = false;
      winchCount++;
      //System.out.println("i tried");
      winchPos = winchCount % 2;
      //System.out.println(winchArray[winchPos]);
      winchController.configureGoal(winchArray[winchPos]-winchEnc.get(), winch_max_v, winch_max_a, true);
    }
    //System.out.println(winchDown.get());

}

  public void hatchLogic(){
    if(!winchDown.get()&&yButton1.held()&&xButton2.held()){
      hatchController.configureGoal(HATCH_EMERGENCY_DOWN-hatchPot.get(), 500, 500, false);
      
    }

    if (hatchPot.get() >= HATCH_SAFE_TOP && hatchPot.get() <= HATCH_SAFE_BOTTOM) {
      if(!hatchController.isEnabled()){
        hatchController.enable();
      }
      if(hatchInButton2.changed() || hatchToggle.changed()){
        hatchDown=!hatchDown;
        if (!hatchDown) {
          hatchController.configureGoal(HATCH_UP-hatchPot.get(), 500, 500, true);
          lb.unlight(hatchToggle);
        }
        else {
          hatchController.configureGoal(HATCH_DOWN-hatchPot.get(), 500, 500, true);
          lb.light(hatchToggle);
        }
      }
    }else{
      hatchController.disable();
      hatch.set(ControlMode.PercentOutput, 0.0);
            //System.out.println("why it do that");
    }
  
  }

  public void dashboardPuts(){
    SmartDashboard.putNumber("hatchPot", hatchPot.get());
    SmartDashboard.putNumber("winchEnc", winchEnc.get());
    SmartDashboard.putNumber("left enc", driveLeftEnc.get());
    SmartDashboard.putNumber("right enc", driveRightEnc.get());
    SmartDashboard.putNumber("left in", leftInInches.pidGet());
    SmartDashboard.putNumber("right in", rightInInches.pidGet());
    //ATLAS
    SmartDashboard.putNumber("elevatorEnc", elevatorEnc.get());
    SmartDashboard.putNumber("elevator set", elevatorController.getSetpoint());
    SmartDashboard.putBoolean("limit switch", winchDown.get());
    //SmartDashboard.putNumber("elevator current", elevator.getOutputCurrent());
  }
  public void calibrateNow(SnazzyMotionPlanner p) {
    if(calibrateButton.changed()&& calibrateButton.on()){
        driveRightEnc.reset();
        driveLeftEnc.reset();
        leftController.enable(); 
        rightController.enable(); 
        leftController.startCalibration();
        rightController.startCalibration();
        /*spinnyController.enable();
        spinnyController.startCalibration();*/
        
        /*
        winchEnc.reset();
        p.enable();
        winchController.startCalibration();
        */
        System.out.println("enabel");
				
    }else if (calibrateButton.changed()&& !calibrateButton.on()){
      
      leftController.disable();
      rightController.disable();
      /*
      p.disable();*/
      //spinnyController.disable();
      System.out.println("disabel");
    }
    //System.out.println(calibrateButton.changed()+" " +calibrateButton.on());
  }
  
  public void pidTuneNow(SnazzyMotionPlanner p) {
    TrajectoryPlanner dynamo;
      //rightController.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));
      //leftController.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));
      if(calibrateButton.on() && calibrateButton.changed()){
        System.out.println("ENABLE");

        /*table = NetworkTableInstance.getDefault().getTable("limelight");
        
        double[] defaultValue = new double[6];
        double[] tarpos = table.getEntry("camtran").getDoubleArray(defaultValue);
        //ystem.out.println(table.getEntry("camtran"));

        //POSITIVE Y IS LEFT, POSITIVE ANGLE IS COUNTERCLOCKWISE

        double[][] dynamoPlan = {{0,0,0},{-tarpos[2]-40, tarpos[0]-6, tarpos[4]}};
        double[][] fakePlan = {{0,0,0},{30, 4, 15}};
        dynamo = new TrajectoryPlanner(fakePlan, 50, 50, 50, "vision.csv");

        System.out.println(-tarpos[2]-40 + ", " + tarpos[0] + ", " + tarpos[4]);

        dynamo.generate();

        System.out.println("GENED UP");*/

        //leftController.configureGoal(SmartDashboard.getNumber("Setpoint", 0.0), 100, 100, false);
        //rightController.configureGoal(SmartDashboard.getNumber("Setpoint", 0.0), 100, 100, false);
            //System.out.println(dynamo.getLeftTrajectory().length());
            driveLeftEnc.reset();
            driveRightEnc.reset();
            //leftController.configureTrajectory(racetrackTurnTraj.getLeftTrajectory(), false);
            //rightController.configureTrajectory(racetrackTurnTraj.getRightTrajectory(), false);

            rightController.enable();
            leftController.enable();
            // System.out.println("enable" + SmartDashboard.getNumber("Setpoint", 0.0));

            //elevator.set(ControlMode.PercentOutput, SmartDashboard.getNumber("Setpoint",0.0));

        }else if (calibrateButton.changed()&& !calibrateButton.on()){
          rightController.disable();
          leftController.disable();
          System.out.println("DISABLE");
          //elevator.set(ControlMode.PercentOutput, 0.0);
      }

      /*if (hatchPot.get() <= 1500 && hatchPot.get() >= 3500) {
        rightController.disable();
          leftController.disable();
        System.out.println("DISABLE");
      }*/
    }
	
  public Trajectory deserializeTraj(){
    Trajectory traj;
    try {
      System.out.println("KAMALA IS A COP");
      System.out.println(SmartDashboard.getString("triplesee", "").length());

      BufferedWriter writer = new BufferedWriter(new FileWriter("/home/lvuser/traj.csv"));
      writer.write(SmartDashboard.getString("triplesee", ""));
      writer.close();
      traj = Pathfinder.readFromCSV(new File("/home/lvuser/traj.csv"));

      System.out.println(traj.length());

    } catch(Exception e) {
      traj = null;
    }
    return traj;
  }
  
  public Trajectory enhanceTrajectory(Trajectory t){
    int i;
    Trajectory outTraj = new Trajectory(((t.length()-1)*10)+1);
    for (i =0; i<t.length()-1;i++){
        for(int j = 0; j<10; j++){
          double newdt = t.segments[i].dt+((t.segments[i+1].dt-t.segments[i].dt)/10)*j;
          double newx = t.segments[i].x+((t.segments[i+1].x-t.segments[i].x)/10)*j;
          double newy = t.segments[i].y+((t.segments[i+1].y-t.segments[i].y)/10)*j;
          double newpos = t.segments[i].position+((t.segments[i+1].position-t.segments[i].position)/10)*j;
          double newv = t.segments[i].velocity+((t.segments[i+1].velocity-t.segments[i].velocity)/10)*j;
          double newaccel = t.segments[i].acceleration+((t.segments[i+1].acceleration-t.segments[i].acceleration)/10)*j;
          double newjerk = t.segments[i].jerk+((t.segments[i+1].jerk-t.segments[i].jerk)/10)*j;
          double newheading = t.segments[i].heading+((t.segments[i+1].heading-t.segments[i].heading)/10)*j;
          outTraj.segments[i*10+j] = new Trajectory.Segment(newdt, newx, newy, newpos, newv, newaccel, newjerk, newheading);
        }
    }
    outTraj.segments[i*10] = t.segments[i];
    return outTraj;
  }

  public boolean isClose(double s, double t, double err){
    if(s <= t+err && s>= t-err){
      return true;
    } else if (t <= s+err && t>= s-err){
      return true;
    }
    else {
      return false;
    }
  }

}
