package org.usfirst.frc4904.robot;

import org.usfirst.frc4904.standard.custom.controllers.CustomCommandJoystick;
import org.usfirst.frc4904.standard.custom.controllers.CustomCommandXbox;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CANTalonFX;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.usfirst.frc4904.robot.subsystems.Intake;
import org.usfirst.frc4904.robot.subsystems.arm.ArmExtensionSubsystem;
import org.usfirst.frc4904.robot.subsystems.arm.ArmPivotSubsystem;
import org.usfirst.frc4904.robot.subsystems.arm.ArmSubsystem;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

// import org.usfirst.frc4904.standard.LogKitten;

import org.usfirst.frc4904.standard.custom.motorcontrollers.CustomCANSparkMax;
import org.usfirst.frc4904.standard.subsystems.motor.SparkMaxMotorSubsystem;
import org.usfirst.frc4904.standard.subsystems.chassis.WestCoastDrive;
import org.usfirst.frc4904.standard.subsystems.motor.TalonMotorSubsystem;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotMap {

    public static class Port {
        // public static class Network {
        //     public static SocketAddress LOCAL_SOCKET_ADDRESS = new InetSocketAddress(3375);
        //     public static SocketAddress LOCALIZATION_ADDRESS = new InetSocketAddress("10.49.04.10", 4321);
        // }

        public static class HumanInput {
            public static final int joystick = 0;
            public static final int xboxController = 1;
        }

        // // blinky constants


        
        // 2023 robot constants
        public static class CANMotor {
            public static final int RIGHT_DRIVE_A = 3;
            public static final int RIGHT_DRIVE_B = 4;
            public static final int LEFT_DRIVE_A = 1;
            public static final int LEFT_DRIVE_B = 2;

            public static final int PIVOT_MOTOR_LEFT = 11;
            public static final int PIVOT_MOTOR_RIGHT = 12;
            public static final int ARM_EXTENSION_MOTOR = 14;

            public static final int LEFT_INTAKE_MOTOR = 21;
            public static final int RIGHT_INTAKE_MOTOR = 22;
        }

        public static class PWM {
        }

        public static class CAN {
        }

        public static class Pneumatics {
        }

        public static class Digital {
        }

       }

    public static class Metrics {
        // blinky constants
        // public static class Chassis {
        //     public static final double GEAR_RATIO = 69/5;
        //     public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
        //     public static final double TRACK_WIDTH_METERS = 0.59;
        // }
        public static final int TALON_ENCODER_COUNTS_PER_REV = 2048;

        // // 2023-robot constants

        public static class Chassis {
            public static final double GEAR_RATIO = 496/45; // https://www.desmos.com/calculator/llz7giggcf
            public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(5);
            public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(19.5); // +/- 0.5 inches
            public static final double CHASSIS_LENGTH = Units.inchesToMeters(37); // +/- 0.5 inches

        }
    }

    public static class PID {
        public static class Drive {
            // PID constants (as of 3/20 characterization)
            public static final double kP = 3.3016;
            public static final double kI = 0;  // FIXME: tune
            public static final double kD = 0;
            // feedforward constants (as of 3/20 characterization)
            public static final double kS = 0.12507; 
            public static final double kV = 2.9669;
            public static final double kA = 0.67699;
        }

        public static class Turn {
        }
    }

    public static class Component {
        //chassis
        public static WPI_TalonFX frontLeftWheelTalon;
        public static WPI_TalonFX frontRightWheelTalon;
        public static WPI_TalonFX backLeftWheelTalon;
        public static WPI_TalonFX backRightWheelTalon;

        //gyro
        public static AHRS navx;


        public static DifferentialDrive chassis;
        
        //arm
        public static ArmSubsystem arm;

        public static ArmPivotSubsystem armPivot;
        public static ArmExtensionSubsystem armExtension;

        public static WPI_TalonFX pivotMotorLeft;
        public static WPI_TalonFX pivotMotorRight;
        public static WPI_TalonFX armExtensionMotor;

        //intake
        public static Intake intake;
        public static CANSparkMax leftIntakeMotor;
        public static CANSparkMax rightIntakeMotor;
    }

    public static class NetworkTables {
        public static NetworkTableInstance instance;

        public static class Odometry {
            public static NetworkTable table;
            public static NetworkTableEntry pose;
            public static NetworkTableEntry accel;
            public static NetworkTableEntry turretAngle;
        }

        public static class Localization {
            public static NetworkTable table;
            public static NetworkTableEntry goalDistance;
            public static NetworkTableEntry goalRelativeAngle;
        }
    }

    public static class Input {
    }

    public static class HumanInput {
        public static class Driver {
            public static CommandXboxController xbox;
        }

        public static class Operator {
            public static CustomCommandJoystick joystick;
        }
    }

    public RobotMap() {
        Component.navx = new AHRS(SerialPort.Port.kMXP);

        HumanInput.Driver.xbox = new CommandXboxController(Port.HumanInput.xboxController);
		HumanInput.Operator.joystick = new CustomCommandJoystick(Port.HumanInput.joystick, 0.1);
        // // UDP things
        // try {
        //     Component.robotUDP = new RobotUDP(Port.Network.LOCAL_SOCKET_ADDRESS, Port.Network.LOCALIZATION_ADDRESS);
        // } catch (IOException ex) {
        //     LogKitten.f("Failed to initialize UDP subsystem");
        //     LogKitten.ex(ex);
        // }


        /***********************
         * Chassis Subsystem
        *************************/
        Component.backLeftWheelTalon = new WPI_TalonFX(Port.CANMotor.LEFT_DRIVE_A);
        Component.backLeftWheelTalon.setNeutralMode(NeutralMode.Brake);

        Component.frontLeftWheelTalon  = new WPI_TalonFX(Port.CANMotor.LEFT_DRIVE_B);
        Component.frontLeftWheelTalon.setNeutralMode(NeutralMode.Brake);

        MotorControllerGroup m_left  = new MotorControllerGroup(Component.frontLeftWheelTalon, Component.backLeftWheelTalon);
        m_left.setInverted(true);


        Component.backRightWheelTalon  = new WPI_TalonFX(Port.CANMotor.RIGHT_DRIVE_A);
        Component.backRightWheelTalon.setNeutralMode(NeutralMode.Brake);

        Component.frontRightWheelTalon = new WPI_TalonFX(Port.CANMotor.RIGHT_DRIVE_B);
        Component.frontRightWheelTalon.setNeutralMode(NeutralMode.Brake);
        
        MotorControllerGroup m_right = new MotorControllerGroup(Component.frontRightWheelTalon, Component.backRightWheelTalon);

        Component.chassis = new DifferentialDrive(
            m_left,
            m_right
        );
        

        /***********************
         * Arm Subsystem
        *************************/

        Component.pivotMotorLeft = new WPI_TalonFX(RobotMap.Port.CANMotor.PIVOT_MOTOR_LEFT);
        Component.pivotMotorLeft.setInverted(true);

        Component.pivotMotorRight = new WPI_TalonFX(RobotMap.Port.CANMotor.PIVOT_MOTOR_RIGHT);

        MotorControllerGroup pivotMotors = new MotorControllerGroup(Component.pivotMotorLeft, Component.pivotMotorRight);

        Component.armExtensionMotor = new WPI_TalonFX(Port.CANMotor.ARM_EXTENSION_MOTOR);

        Component.armExtension = new ArmExtensionSubsystem(
            Component.armExtensionMotor,
            () -> ArmPivotSubsystem.motorRevsToAngle(Component.pivotMotorRight.getSelectedSensorPosition())
        );
        // Autonomous.autonCommand = Component.chassis.c_buildPathPlannerAuto(
        //     PID.Drive.kS, PID.Drive.kV, PID.Drive.kA,
        //     Autonomous.RAMSETE_B, Autonomous.RAMSETE_ZETA,
        //     Autonomous.AUTON_NAME, Autonomous.MAX_VEL, Autonomous.MAX_ACCEL,
        //     Autonomous.autonEventMap
        // );

        Component.armPivot = new ArmPivotSubsystem(Component.pivotMotorLeft, Component.pivotMotorRight, Component.armExtension::getCurrentExtensionLength);

        Component.arm = new ArmSubsystem(Component.armPivot, Component.armExtension);

        /***********************
         * Intake Subsystem
        *************************/
        Component.leftIntakeMotor = new CANSparkMax(Port.CANMotor.LEFT_INTAKE_MOTOR, MotorType.kBrushless);
        Component.rightIntakeMotor = new CANSparkMax(Port.CANMotor.RIGHT_INTAKE_MOTOR, MotorType.kBrushless);
        Component.intake = new Intake(Component.leftIntakeMotor, Component.rightIntakeMotor);

                
        // links we'll need
        // - angles and distances for intake/outtake: https://docs.google.com/spreadsheets/d/1B7Ie4efOpuZb4UQsk8lHycGvi6BspnF74DUMLmiKGUM/edit?usp=sharing
        // - naive + scuffed ramsete tuning: https://docs.google.com/spreadsheets/d/1BIvwJ6MfLf9ByW9dcmagXFvm7HPaXY78Y4YB1L9TGPA/edit#gid=0
    }
}