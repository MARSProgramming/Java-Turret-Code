package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Turret extends SubsystemBase {
    private TalonFX flywheelMaster;
    private TalonFX flywheelFollower;
    private TalonFX hood;
    private TalonSRX magazine;
    private TalonSRX turretMotor;
    
    public Turret() {
        flywheelMaster = new TalonFX(Constants.TURRET_FRONT_FLYWHEEL_MOTOR);
        flywheelFollower = new TalonFX(Constants.TURRET_BACK_FLYWHEEL_MOTOR);
        hood = new TalonFX(Constants.TURRET_HOOD_MOTOR);

        magazine = new TalonSRX(Constants.TURRET_MAGAZINE);
        turretMotor = new TalonSRX(Constants.TURRET_MOTOR);

        flywheelMaster.configFactoryDefault();
        flywheelFollower.configFactoryDefault();
        hood.configFactoryDefault();
        magazine.configFactoryDefault();
        turretMotor.configFactoryDefault();

        flywheelMaster.config_kP(0, 0);
        flywheelMaster.config_kI(0,0); 
        flywheelMaster.config_kD(0,0);
        flywheelMaster.config_kF(0,0);
        
        hood.config_kP(0,0); 
        hood.config_kI(0,0); 
        hood.config_kD(0,0); 
        hood.configForwardSoftLimitThreshold(Constants.TURRET_HOOD_FORWARD_LIMIT);
        hood.configForwardSoftLimitEnable(true);
        hood.configReverseSoftLimitThreshold(Constants.TURRET_HOOD_REVERSE_LIMIT);
        hood.configReverseSoftLimitEnable(true);


        flywheelMaster.setNeutralMode(NeutralMode.Coast);
        flywheelFollower.setNeutralMode(NeutralMode.Coast);

        turretMotor.setNeutralMode(NeutralMode.Brake);
        magazine.setNeutralMode(NeutralMode.Coast);
        hood.setNeutralMode(NeutralMode.Brake);

        flywheelFollower.setInverted(false);
        flywheelMaster.setInverted(false);

        flywheelFollower.follow(flywheelMaster);
    }

    public CommandBase setFlywheelOutput(double percent) {
        return runEnd(() -> {
            flywheelMaster.set(ControlMode.PercentOutput, percent);
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
            hood.set(ControlMode.Position, position);
        },
        () -> {
            hood.set(ControlMode.PercentOutput, 0);
        });
    }

    public CommandBase setTurretMotorOutput(double percent) {
        return runEnd(() -> {
            turretMotor.set(ControlMode.PercentOutput, percent);
        }, () -> {
            turretMotor.set(ControlMode.PercentOutput, 0);
        });
    }

    public CommandBase setTurretMotorVelocity(double velocity) {
        return runEnd(() -> {
            turretMotor.set(ControlMode.PercentOutput, velocity);
        }, () -> {
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

    public CommandBase setMagazineVelocity(double velocity) {
        return runEnd(() -> {
            magazine.set(ControlMode.PercentOutput, velocity);
        }, () -> {
            magazine.set(ControlMode.PercentOutput, 0);
        });
    }

}
