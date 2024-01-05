package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Turret extends SubsystemBase {
    private TalonFX flywheelMaster;
    private TalonFX flywheelFollower;
    private TalonFX hood;
    private TalonSRX magazine;
    private TalonSRX turretMotor;
    private double hoodGearRatio = 15.0/85.0;
    private double hoodNativePositionToRadians = 2 * Math.PI / 2048 * hoodGearRatio;
    private double turretGearRatio = 1.0/100.0 * 180/40;
    private double turretNativePositionToDegrees = 360.0 / 2048 * turretGearRatio;

    public Turret() {
        
        flywheelMaster = new TalonFX(Constants.TURRET_FRONT_FLYWHEEL_MOTOR);
        flywheelFollower = new TalonFX(Constants.TURRET_BACK_FLYWHEEL_MOTOR);
        hood = new TalonFX(Constants.TURRET_HOOD_MOTOR);

        magazine = new TalonSRX(Constants.TURRET_MAGAZINE);
        turretMotor = new TalonSRX(Constants.TURRET_MOTOR);

        Timer myTimer = new Timer();
        myTimer.reset();

        flywheelMaster.configFactoryDefault();
        flywheelFollower.configFactoryDefault();
        hood.configFactoryDefault();
        magazine.configFactoryDefault();
        turretMotor.configFactoryDefault();
        flywheelMaster.configClosedloopRamp(0.25);
        flywheelMaster.config_kP(0, 0.1);
        flywheelMaster.config_kI(0,0); 
        flywheelMaster.config_kD(0,0);
        flywheelMaster.config_kF(0,0.048);
        
        turretMotor.setSelectedSensorPosition(0);
        turretMotor.configForwardSoftLimitThreshold(Constants.TURRET_FORWARD_LIMIT / turretNativePositionToDegrees);
        turretMotor.configForwardSoftLimitEnable(true);
        turretMotor.configReverseSoftLimitThreshold(Constants.TURRET_REVERSE_LIMIT / turretNativePositionToDegrees);
        turretMotor.configReverseSoftLimitEnable(true);
        turretMotor.configPeakOutputForward(Constants.TURRET_FORWARD_PEAK_OUTPUT);
        turretMotor.configPeakOutputReverse(Constants.TURRET_REVERSE_PEAK_OUTPUT);
        turretMotor.config_kP(0, 0.3);
        turretMotor.config_kI(0,0); 
        turretMotor.config_kD(0,0);

        hood.config_kP(0,0.25); 
        hood.config_kI(0,0); 
        hood.config_kD(0,0); 
        hood.setSelectedSensorPosition(0);
        hood.configForwardSoftLimitThreshold(Constants.TURRET_HOOD_FORWARD_LIMIT);
        hood.configForwardSoftLimitEnable(true);
        hood.configReverseSoftLimitThreshold(Constants.TURRET_HOOD_REVERSE_LIMIT);
        hood.configReverseSoftLimitEnable(true);
        hood.configPeakOutputForward(Constants.TURRET_HOOD_FORWARD_PEAK_OUTPUT);
        hood.configPeakOutputReverse(Constants.TURRET_HOOD_REVERSE_PEAK_OUTPUT);

        magazine.config_kP(0, 0.15);
        magazine.config_kI(0,0); 
        magazine.config_kD(0,0);
        magazine.config_kF(0,0.11);

        hood.configPeakOutputForward(0.1);
        hood.configPeakOutputReverse(0.1);

        flywheelMaster.setNeutralMode(NeutralMode.Coast);
        flywheelFollower.setNeutralMode(NeutralMode.Coast);
        turretMotor.setNeutralMode(NeutralMode.Brake);
        magazine.setNeutralMode(NeutralMode.Coast);
        hood.setNeutralMode(NeutralMode.Brake);

        flywheelFollower.setInverted(true);
        flywheelMaster.setInverted(true);
        magazine.setSensorPhase(true);

        //flywheelFollower.follow(flywheelMaster);
    }

    public CommandBase setFlywheelOutput(DoubleSupplier percent) {
 
        return runEnd(() -> {
            flywheelMaster.set(ControlMode.PercentOutput, percent.getAsDouble());
        },
        () -> {
            flywheelMaster.set(ControlMode.PercentOutput, 0);
        }
        );
    }

    public CommandBase setFlywheelVelocity(double velocity) {
        return runEnd(() -> {
            flywheelMaster.set(ControlMode.Velocity, velocity);
        },
        () -> {
            flywheelMaster.set(ControlMode.PercentOutput, 0);
        });
    }

    public CommandBase setHoodOutput(double percent) {
        return runEnd(() -> {
            hood.set(ControlMode.PercentOutput, percent);
            
        },
        () -> {
            hood.set(ControlMode.PercentOutput, 0);
        }
        );
    } 
    

    public CommandBase setHoodPosition(double position) {
        return runEnd(() -> {
            hood.set(ControlMode.Position, position / hoodNativePositionToRadians);
        },
        () -> {
            hood.set(ControlMode.PercentOutput, 0);
        });
    }

    public CommandBase setTurretOutput(double percent) {
        return runEnd(() -> {
            turretMotor.set(ControlMode.PercentOutput, percent);
        },
        () -> {
            turretMotor.set(ControlMode.PercentOutput, 0);
        }
        );
    } 
    

    public CommandBase setTurretPosition(double position) {
        return runEnd(() -> {
            turretMotor.set(ControlMode.Position, position / turretNativePositionToDegrees);
        },
        () -> {
            turretMotor.set(ControlMode.PercentOutput, 0);
        });
    }

    public CommandBase setMagazineOutput(double percent) {
        return runEnd(() -> {
            magazine.set(ControlMode.PercentOutput, percent);
        }, () -> {
            magazine.set(ControlMode.PercentOutput, 0);
        });
    }

    public CommandBase shootball(double magazinePercent, double flywheelPercent) {
        return runEnd(()-> {
            magazine.set(ControlMode.PercentOutput, magazinePercent);
            flywheelMaster.set(ControlMode.PercentOutput, flywheelPercent);
            if (hood.getSelectedSensorPosition() < 500) {
                hood.set(ControlMode.PercentOutput, 0.1);
            } else {
                hood.set(ControlMode.PercentOutput, 0);
            }
        }, () -> {
            magazine.set(ControlMode.PercentOutput, 0);
            flywheelMaster.set(ControlMode.PercentOutput, 0);
            
        });
    }

    public CommandBase setMagazineVelocity(double velocity) {
        return runEnd(() -> {
            magazine.set(ControlMode.Velocity, velocity);
        }, () -> {
            magazine.set(ControlMode.PercentOutput, 0);
        });
    }

    public CommandBase testShoot(DoubleSupplier x, DoubleSupplier y){
        return runEnd(() -> {
            magazine.set(ControlMode.Velocity, 3000);
            flywheelMaster.set(ControlMode.Velocity, 7000);
            hood.set(ControlMode.Position, 9 / hoodNativePositionToRadians);
        }, () -> {
            magazine.set(ControlMode.PercentOutput, 0);
            flywheelMaster.set(ControlMode.PercentOutput, 0);
            hood.set(ControlMode.Position, 0);
        });
    }

    @Override
    public void periodic(){ 
        SmartDashboard.putNumber("Flywheel Velocity", flywheelMaster.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Magazine Velocity", magazine.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Turret Position", turretMotor.getSelectedSensorPosition() * turretNativePositionToDegrees);
        SmartDashboard.putNumber("Hood Position", hood.getSelectedSensorPosition() * hoodNativePositionToRadians);
    }
}
