package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ThreeStageViper{

    Servo threeStageLeft, threeStageRight;

  public ThreeStageViper(HardwareMap map) {
      threeStageLeft = map.get(Servo.class,"three_stage_left_viper");
      threeStageRight = map.get(Servo.class,"three_stage_right_viper");

      threeStageLeft.setDirection(Servo.Direction.REVERSE);
      threeStageRight.setDirection(Servo.Direction.REVERSE);
  }

  public class ThreeStageExtend implements Action {

      double extend = 0.50;
      @Override
      public boolean run(@NonNull TelemetryPacket telemetryPacket) {
          threeStageLeft.setPosition(extend);
          threeStageRight.setPosition(extend);
          return false;
      }
  }

  public class ThreeStageRetract implements Action {

      double retract = 0.08;
      @Override
      public boolean run(@NonNull TelemetryPacket telemetryPacket) {

          threeStageLeft.setPosition(retract);
          threeStageRight.setPosition(retract);
          return false;
      }
  }

  public Action threeStageExtend(){
      return new ThreeStageExtend();
  }

  public Action threeStageRetract() {
      return new ThreeStageRetract();
  }
}




