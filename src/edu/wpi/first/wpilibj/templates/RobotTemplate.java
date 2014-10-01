/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.templates;

//import edu.wpi.first.wpilibj.IterativeRobot;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
//import edu.wpi.first.wpilibj.RobotDrive;
//import edu.wpi.first.wpilibj.Solenoid;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Victor;
//import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.CANJaguar;
//import edu.wpi.first.wpilibj.Relay;
//import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends IterativeRobot
{

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */

    /*
     * setting up variables for changing values of PID constants & max speed in smart dashboard
     */
    Preferences prefs = Preferences.getInstance();

    double P;
    double I;
    double D;
    double MAX_RPM;
    double StartPosition;

    double[] motorSpeed = new double[4]; //holds motor speeds (in rpm)

    int m_dsPacketsIntakedInCurrentSecond;	// keep track of the ds packets received in the current second

    // Declare variables for the driver joysticks
    Joystick JoyLeft = new Joystick(1);			// joystick 1 (arcade stick or right tank stick)
    Joystick JoyRight = new Joystick(2);                   //joystick 2 (arcade stick or left tank stick)
    JoystickButton DriverIntake = new JoystickButton(JoyRight, 2);   //starts arms intake, joystick 1, button 3
    JoystickButton DriverEject = new JoystickButton(JoyLeft, 2);     //starts arms reject, joystick 2, button 3
    JoystickButton CrabOn = new JoystickButton(JoyLeft, 8); //turn crabbing on [using mecanums]
    JoystickButton CrabOff = new JoystickButton(JoyLeft, 9); //turn crabbing off [using traction]

    boolean CrabState = true;   //determines if crabbing is on or off
    int autonState = 0;
    double autonIngestTime;
    Timer autonTimer = new Timer();
    double driveTime;
    // Declare variables for the operator joystick
    Joystick JoyOp = new Joystick(3);
    JoystickButton OpIntake = new JoystickButton(JoyOp, 3);   //Press-and-hold for roller intake
    JoystickButton OpEject = new JoystickButton(JoyOp, 2);     //Press-and-hold for roller eject
    JoystickButton OpAutoIntake = new JoystickButton(JoyOp, 1);   //starts automatic ball intake
    JoystickButton OpDeploy = new JoystickButton(JoyOp, 8);
    JoystickButton OpReloadPrefs = new JoystickButton(JoyOp, 12);   //use this button when changing values in SmartDashboard 
    

    CANJaguar motor1;       //ctrls the drive motors
    CANJaguar motor2;
    CANJaguar motor3;
    CANJaguar motor4;
    DigitalInput cageLimitSwitch;
    DigitalInput ballLimitSwitch;
    AnalogPotentiometer winchlength;
    DigitalInput autonSwitch;
    Victor Winch = new Victor(5);        //ctrls the cage and intake roller motion
    Victor Ingest = new Victor(6);
    int CANTimeouts;
    int m_autoPeriodicLoops;
    int m_disabledPeriodicLoops;
    int m_telePeriodicLoops;

    int ingestState = 0; //check what ingest is doing
    int cageState = 0; //check what cage is doing
    int cageCommand = 0; //tell cage want it needs to do

    final double PI = 3.141592654;
    final double Maxwinch = 3.0;

    public RobotTemplate()
    {
    }

    public void CANTimeout()
    {
        CANTimeouts++;
        SmartDashboard.putNumber("CANTimeouts", CANTimeouts);
    }

    /*
     * keeps track of the ingest states & transitions between them
     * 
     */
    public void controlIngest()
    {

        switch (ingestState)
        {

            case 0: //no ingest in progress
                //check button & advance to state 1 if pressed
                Ingest.set(0);  //rollers are stopped
                if (OpAutoIntake.get())
                {
                    ingestState = 1;
                }
                break;
            case 1:
                cageCommand = 1;// extend cage
                Ingest.set(1); //makes rollers pick up ball
                if (cageState == 1)
                {
                    ingestState = 2;
                }
                break;
            case 2:
                Ingest.set(1); //ingest
                boolean bl = !ballLimitSwitch.get();
                if (bl)
                {
                    ingestState = 3;
                }
                break;
            case 3:
                cageCommand = -1; //retract cage
                Ingest.set(0); 
                if (cageState == -1)
                {
                    ingestState = 4;
                    autonTimer.reset();
                    autonTimer.start();
                }
                break;
            case 4:
                Ingest.set(1);
                if(autonTimer.get() > 0.75)
                {
                    Ingest.set(0);
                    autonTimer.stop();
                }
                break;
            default:
                System.out.println("Invalid Ingest State" + ingestState);
                ingestState = 0;
        }
    }

    /*
     * sets the ranges for controlling how fast to run the winch motor
     */
    public void setWinch(double speed)
    {
        double len = winchlength.get(); //need to display winchlength to find l
        if ((len < 1.0) || (len > 4.0))
        {
            speed = 0.0;
        } else
        {
            if ((len < 1.2) && (speed > 0.0))
            { //pot values: greater = in, smaller = out
                speed = 0.0;
            } else
            {
                if ((len > 3.9) && (speed < 0.0))
                {
                    speed = 0.0;
                }
            }
        }
        Winch.set(speed); //positive values = out
    }

    /*
     * changes the states of the cage based on the position of the winch
     */
    public void controlCage()
    {

        double len = winchlength.get(); //need to display winchlength to find l

        if (len > 3.6 || !(cageLimitSwitch.get()))
        {   //get numbers of these later
            cageState = -1;
        } else
        {
            if (len < 1.7)
            {
                cageState = 1;
            } else
            {
                cageState = 0;
            }
        }

        switch (cageCommand)
        {
            case 1: //we want it to go out
                if (len > 2.6)
                {
                    setWinch(0.2); // maximum unwind power
                } else
                {
                    if (len > 1.6)
                    {
                        setWinch((len - 1.6) / 5); // soft landing
                    } else
                    {
                        setWinch(0.0);
                    }
                }
                break;
            case -1: //want it to go in
                if (cageState == -1)
                { //if the cage is already all the way in
                    setWinch(len - 3.8);//so it overcomes the elastic
                } else
                {
                    setWinch(-0.35);//else if it is not all the way in set the speed
                }
                break;
            case -2:
                if (!cageLimitSwitch.get())
                {
                    cageCommand = -1;
                } else
                {
                    setWinch(-0.35);
                }
                break;
            default:
                System.out.println("Invalid case value" + cageCommand);
                cageCommand = -1;
        }

    }

    public void robotInit()
    {

        SmartDashboard.putNumber("CAN timeouts", CANTimeouts);
        boolean CANInit = false;
        CANTimeouts = 0;
        while (CANInit == false)
        {
            try
            {
//                m_telePeriodicLoops = 0;				// Reset the number of loops in current second
//                m_dsPacketsReceivedInCurrentSecond = 0;                 // Reset the number of dsPackets in current second

                motor1 = new CANJaguar(2, CANJaguar.ControlMode.kSpeed); //determines that the Jaguar controls for speed
                motor2 = new CANJaguar(4, CANJaguar.ControlMode.kSpeed);
                motor4 = new CANJaguar(8, CANJaguar.ControlMode.kSpeed);
                motor3 = new CANJaguar(6, CANJaguar.ControlMode.kSpeed);
                cageLimitSwitch = new DigitalInput(1);
                ballLimitSwitch = new DigitalInput(2);
                autonSwitch = new DigitalInput(3);
                winchlength = new AnalogPotentiometer(2);
                motor1.setPositionReference(CANJaguar.PositionReference.kQuadEncoder);    //chooses which kind of encoder to determine speed feedback
                motor1.setSpeedReference(CANJaguar.SpeedReference.kQuadEncoder);    //chooses which kind of encoder to determine speed feedback
                motor2.setSpeedReference(CANJaguar.SpeedReference.kQuadEncoder);
                motor3.setSpeedReference(CANJaguar.SpeedReference.kQuadEncoder);
                motor4.setSpeedReference(CANJaguar.SpeedReference.kQuadEncoder);
                motor1.configMaxOutputVoltage(15);
                motor2.configMaxOutputVoltage(15);
                motor3.configMaxOutputVoltage(15);
                motor4.configMaxOutputVoltage(15);
                motor1.configEncoderCodesPerRev(256);   //counts pulses per revolution 
                motor2.configEncoderCodesPerRev(256);
                motor3.configEncoderCodesPerRev(256);
                motor4.configEncoderCodesPerRev(256);
                StartPosition = motor1.getPosition();

                updatePrefs();

                motor1.setX(0);
                motor2.setX(0);
                motor3.setX(0);
                motor4.setX(0);

                motor1.enableControl(); //starts feedback ctrl
                motor2.enableControl();
                motor3.enableControl();
                motor4.enableControl();
                CANInit = true;
            } catch (CANTimeoutException ex)
            {
                CANTimeout();
            }
        }
        System.out.println("### PTERODACTYL HAS ARRIVED ###");

    }

    /**
     * This function is called once before autonomous control
     */
    public void disabledInit()
    {
        System.out.println("### DISABLED ###");
    }

    /**
     * This function is called periodically during autonomous
     */
    public void disabledPeriodic()
    {
    }

    /**
     * This function is called once before autonomous control
     */
    public void autonomousInit()
    {
        System.out.println("### AUTONOMOUS MODE ENABLED ###");
        cageCommand = -2;
        autonState = 4;
        autonIngestTime = autonTimer.getUsClock();
        if (autonSwitch.get())
        {
            autonState = 4;
        } else
        {
            autonState = 0;
        }
    }

    /**
     * This function is called periodically during autonomous
     * This function picks up the ball and moves about 15 feet.
     */
    public void autonomousPeriodic()
    {
        controlCage();
        switch (autonState)
        {
            case 0:
                try
                {
                    motor1.setX(0);
                    motor2.setX(0);
                    motor3.setX(0);
                    motor4.setX(0);
                    Ingest.set(0);
                } catch (CANTimeoutException ex)
                {
                    CANTimeout();
                }
                break;
            case 1: // Start cage deployment
                cageCommand = -2;
                controlIngest();
                autonState = 2;
                break;
            case 2: // Wait for cage to finish deploying
                controlIngest();
                if (cageCommand == 0)
                {
                    ingestState = 1;
                }
                autonState = 4;
                break;
            case 3: // Cage is deployed, ball capture has started
                if (!ballLimitSwitch.get())
                {
                    autonIngestTime = autonTimer.getUsClock();
                    autonState = 4;
                }
                break;
                
            case 4:
                try {
                    StartPosition = motor1.getPosition();
                    autonState = 5;
                } catch (CANTimeoutException ex)
                {
                    CANTimeout();
                }
                break;
                
            case 5:
                driveTime = autonTimer.getUsClock() - autonIngestTime;
                double Distance = 0.0;
                try
                {
                    motor1.setX(MAX_RPM * 0.5);
                    motor2.setX(-MAX_RPM * 0.5);
                    motor3.setX(-MAX_RPM * 0.5);
                    motor4.setX(MAX_RPM * 0.5);
                    Distance = Math.abs(motor1.getPosition()-StartPosition);
                } catch (CANTimeoutException ex)
                {
                    CANTimeout();
                }
                if (driveTime > 2600000)
                //if (Distance > 10.0);
                {   //change this
                    autonState = 6;
                }
                break;


            case 6:
                System.out.println("we have reached case 5 where release should be made");
                    try
                    {
                        motor1.setX(MAX_RPM * 0);
                        motor2.setX(-MAX_RPM * 0);
                        motor3.setX(-MAX_RPM * 0);
                        motor4.setX(MAX_RPM * 0);
                    } catch (CANTimeoutException ex)
                        {
                            CANTimeout();
                        }
                    
                autonState = 7;
                break;
            
            case 7:
            {
                //autonTimer.reset();
                //autonTimer.start();
                try
                {
                    if (Math.abs(motor2.getSpeed()) < 10)
                    {
                        //autonTimer.stop();
                        autonState = 8;
                    }
                }
                catch (CANTimeoutException ex)
                {
                    CANTimeout();
                }   
                
                break;
            }
            
            
            case 8:
                Ingest.set(-1.0);   //rollers release the ball for the remaining time of autonomous
                break;
                
        }
    }
    
    /*
    This function runs in autonomous mode. The robot moves about 10 feet and scores into the goal
    */
    
    public void autonomousScore()
    {
                autonTimer.reset();
                autonTimer.start();
                
                try
                {
                    motor1.setX(MAX_RPM * 0.5);
                    motor2.setX(-MAX_RPM * 0.5);    //The robot starts moving
                    motor3.setX(-MAX_RPM * 0.5);
                    motor4.setX(MAX_RPM * 0.5);
                }catch (CANTimeoutException ex)
                {
                    CANTimeout();
                }
                
                if(autonTimer.get() > 1.75)   // after 1.75 seconds the robot turns off the motors
                {
                    try
                    {
                        motor1.setX(MAX_RPM * 0);
                        motor2.setX(-MAX_RPM * 0);
                        motor3.setX(-MAX_RPM * 0);
                        motor4.setX(MAX_RPM * 0);
                    } catch (CANTimeoutException ex)
                        {
                            CANTimeout();
                        }
                    autonTimer.stop();
                }
                
                Ingest.set(-1.0);   //rollers release the ball for the remaining time of autonomous
    }

    /**
     * This function is called once before autonomous control
     */
    public void testInit()
    {
        System.out.println("### TEST MODE ENABLED ###");
        cageCommand = -2;
    }

    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic()
    {
    }

    /**
     * This function is called once before operator control
     */
    public void teleopInit()
    {
        System.out.println("### TELEOP MODE ENABLED ###");
        try
        {
            motor1.enableControl(); //starts feedback ctrl
            motor2.enableControl();
            motor3.enableControl();
            motor4.enableControl();
        } catch (CANTimeoutException ex)
        {
            CANTimeout();
        }
        cageCommand = -2;
    }

    // returns speed in ft/s
    public double RobotSpeed(double[] s)
    { //get average of all motors and convert to ft/s
        double sHighest = 0;
        for (int x = 0; x < 4; x++)
        {
            sHighest += s[x];
        }
        sHighest /= 4;

        double speed = (sHighest * PI) / (120);
        return speed;
    }

    /*
     * smartdashboard ctrls
     */
    public void updatePrefs()
    {
        MAX_RPM = prefs.getDouble("M", 0.0);
        P = prefs.getDouble("P", 0.0);   //can change values from here, press button to activate changes
        I = prefs.getDouble("I", 0.0);
        D = prefs.getDouble("D", 0.0);
        SmartDashboard.putNumber("Jaguar P", P);  //displays PID values on SmartDash
        SmartDashboard.putNumber("Jaguar I", I);
        SmartDashboard.putNumber("Jaguar D", D);
        SmartDashboard.putNumber("MAX_RPM", MAX_RPM);
        try
        {
            motor1.setPID(P, I, D);  //sets PID constants for Jag PID loop
            motor2.setPID(P, I, D);
            motor3.setPID(P, I, D);
            motor4.setPID(P, I, D);
            motor1.enableControl(); //starts feedback ctrl
            motor2.enableControl();
            motor3.enableControl();
            motor4.enableControl();
        } catch (CANTimeoutException ex)
        {
            CANTimeout();
        }
        System.out.println("finished prefs");
    }

    /**
     * This function is called periodically during operator control This
     * function is responsible for joystick inputs and motor speed
     */
    public void teleopPeriodic()
    {
        controlCage();
        controlIngest();
        try
        {
            double VertL = -JoyLeft.getY();  //gets vertical & horizontal values from left joystick
            double Horz = (JoyLeft.getX() + JoyRight.getX()) / 2;
            double VertR = -JoyRight.getY(); //gets vertical value from right joystick
            double JoyKneeOne = 0.05;        //first knee of joystick range which starts 'maneuvering range'
            double JoyKneeTwo = 0.8;         //second knee of joystick range which ends 'maneuvering range' and starts 'speed range'
            double JoyMaxRange = 1.0;        //maximum input range of joysticks

            if (Math.abs(VertL) < JoyKneeOne) //implements deadzones
            {
                VertL = 0.0;
            }

            if (Math.abs(VertR) < JoyKneeOne) //implements deadzones
            {
                VertR = 0.0;
            }

            if ((Math.abs(VertL) >= JoyKneeOne) && (Math.abs(VertL) <= JoyKneeTwo)) //mapping for maneuvering range
            {
                if (VertL < 0.0)
                {
                    VertL = (3.0 / 5.0) * VertL - 0.02;     //changes raw negative input into a maneuverable speed
                } else
                {
                    VertL = (3.0 / 5.0) * VertL + 0.02;     //changes raw positive input into a maneuverable speed
                }
            } else
            {
                if ((Math.abs(VertL) > JoyKneeTwo) && (Math.abs(VertL) <= JoyMaxRange)) //mapping for speed range
                {
                    if (VertL < 0)
                    {
                        VertL = (5.0 / 2.0) * VertL + 1.5;  //changes raw negative input into a fast speed
                    } else
                    {
                        VertL = (5.0 / 2.0) * VertL - 1.5;  //changes raw positive input into a fast speed
                    }
                }
            }

            if ((Math.abs(VertR) >= JoyKneeOne) && (Math.abs(VertR) <= JoyKneeTwo)) //mapping for maneuvering range
            {
                if (VertR < 0.0)
                {
                    VertR = (3.0 / 5.0) * VertR - 0.02;
                } else
                {
                    VertR = (3.0 / 5.0) * VertR + 0.02;
                }
            } else
            {
                if ((Math.abs(VertR) > JoyKneeTwo) && (Math.abs(VertR) <= JoyMaxRange)) //mapping for speed range
                {
                    if (VertR < 0.0)
                    {
                        VertR = (5.0 / 2.0) * VertR + 1.5;
                    } else
                    {
                        VertR = (5.0 / 2.0) * VertR - 1.5;
                    }
                }
            }

            //double maxspeed = VertL + VertR;
            //deadband for if not crabbing
            if (Math.abs(Horz) < 0.25)
            {
                Horz = 0.0;
            } //deadband for crabbing
            else
            {
                if (Math.abs(VertL) + Math.abs(VertR) < 1)
                {
                    VertL = 0;
                    VertR = 0;
                }
            }
            /*
             if (maxspeed > 1.8) {
             VertL = 1.0;
             VertR = 1.0;
             Horz = 0;
             } else if (maxspeed < -1.8) {
             VertL = -1.0;
             VertR = -1.0;
             Horz = 0;
             }
             */
            if (CrabOn.get() == true)
            {       //crabbing states
                CrabState = true;
            }
            if (CrabOff.get() == true)
            {
                CrabState = false;
            }
            if (CrabState == false)
            {
                Horz = 0.0;
            }

            // Joystick values are final:  Horz, VertL, VertR (-1..0..1)
            //holonomic code
            double motor1Speed = +VertL - Horz;
            double motor2Speed = -VertR - Horz;
            double motor3Speed = -VertR + Horz;
            double motor4Speed = +VertL + Horz;

            double biggestValue;
            biggestValue = 1.0;

            if (Math.abs(motor1Speed) > 1.0)
            {
                biggestValue = Math.abs(motor1Speed);
            }
            if (Math.abs(motor2Speed) > biggestValue)
            {
                biggestValue = Math.abs(motor2Speed);
            }                                   //finds the biggest motor value
            if (Math.abs(motor3Speed) > biggestValue)
            {
                biggestValue = Math.abs(motor3Speed);
            }
            if (Math.abs(motor4Speed) > biggestValue)
            {
                biggestValue = Math.abs(motor4Speed);
            }

            motor1Speed /= biggestValue;
            motor2Speed /= biggestValue;
            motor3Speed /= biggestValue;
            motor4Speed /= biggestValue;

            // motor#Speed values are final (-1..0..1)
            motor1.setX(MAX_RPM * motor1Speed);
            motor2.setX(MAX_RPM * motor2Speed);
            motor3.setX(MAX_RPM * motor3Speed);
            motor4.setX(MAX_RPM * motor4Speed);

            // Motora are commanded between -500 and 500 RPM
            // Operator runs star rollers with thumb buttons (press-and-hold)
            if (OpIntake.get() || DriverIntake.get())
            {  //if pressed take ball in
                ingestState = 0;
                Ingest.set(1.0);
            } else
            {
                if (OpEject.get() || DriverEject.get())
                {       //if pressed eject ball
                    ingestState = 0;
                    Ingest.set(-1.0);
                } else
                {
                    if (ingestState == 0)
                    {
                        Ingest.set(0.0);
                    }
                }
            }

            // Operator controls cage by joystick fwd/back (Y axis)
            double OpWench = JoyOp.getY();
            if (OpWench > 0.9)
            {
                ingestState = 0;  // Cancel auto-ingest if started
                cageCommand = -1; // Request cage retract
            } else
            {
                if (OpWench < -0.9)
                {
                    ingestState = 0;  // Cancel auto-ingest if started
                    cageCommand = 1;  // Request cage extend
                }
            }

            // Operator can start auto-capture with trigger
            if (OpAutoIntake.get())
            {
                ingestState = 1;
            }

            // Emergency deployment button
            if (OpDeploy.get())
            {
                cageCommand = -2;
            }

            motorSpeed[0] = motor1.getSpeed();      //gets motor speeds
            motorSpeed[1] = motor2.getSpeed();
            motorSpeed[2] = motor3.getSpeed();
            motorSpeed[3] = motor4.getSpeed();

            double command1Speed = motor1Speed;
            double command2Speed = motor2Speed;
            double command3Speed = motor3Speed;
            double command4Speed = motor4Speed;

            SmartDashboard.putNumber("Motor1 Position", Math.abs(motor1.getPosition()-StartPosition));  //displays motor position on SmartDash
            SmartDashboard.putNumber("Motor1 Speed", motorSpeed[0]);  //displays motor speed on SmartDash
            SmartDashboard.putNumber("Motor2 Speed", motorSpeed[1]);
            SmartDashboard.putNumber("Motor3 Speed", motorSpeed[2]);
            SmartDashboard.putNumber("Motor4 Speed", motorSpeed[3]);
            SmartDashboard.putNumber("Commanded Motor1 Speed", command1Speed);
            SmartDashboard.putNumber("Commanded Motor2 Speed", command2Speed);
            SmartDashboard.putNumber("Commanded Motor3 Speed", command3Speed);
            SmartDashboard.putNumber("Commanded Motor4 Speed", command4Speed);
            SmartDashboard.putNumber("Robot Speed", RobotSpeed(motorSpeed));
            SmartDashboard.putNumber("Left Joystick", JoyLeft.getY());
            SmartDashboard.putNumber("Right Joystick", JoyRight.getY());
            SmartDashboard.putNumber("Left Drive", VertL);
            SmartDashboard.putNumber("Right Drive", VertR);
            SmartDashboard.putNumber("Pot Value", winchlength.get());
        } catch (CANTimeoutException ex)
        {
            CANTimeout();
        }

        if (OpReloadPrefs.get() == true)
        {
            updatePrefs();
        }
    }
}
