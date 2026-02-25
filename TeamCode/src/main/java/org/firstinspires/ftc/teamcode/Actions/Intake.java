package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    CRServo intake;

    public Intake(HardwareMap map) {
        intake = map.get(CRServo.class,"intake");
    }
     public class IntakeCollect implements Action {

        @Override
         public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setPower(1.0);
            return false;
        }
     }

     public Action intakeCollect() {
        return new IntakeCollect();
     }
}
