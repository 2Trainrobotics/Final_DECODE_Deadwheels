package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ShooterSubsystem;

public class Shooter {
    ShooterSubsystem shooter;

    public Shooter(ShooterSubsystem shooter) {
        this.shooter = shooter;
    }



    public class ShootClose implements Action {



        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            shooter.setShooterVelocity(1250);
            return false;
        }
    }

    public class ShootFar implements Action {



        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            shooter.setShooterVelocity(1500);
            return false;
        }
    }

    public class ShooterOff implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            shooter.setShooterVelocity(0);
            return false;
        }
    }

    public Action shootFar() {
        return new ShootFar();
    }
    public Action shootClose() {
        return new ShootClose();
    }
    public Action shooterOff() {
        return new ShooterOff();
    }
}



