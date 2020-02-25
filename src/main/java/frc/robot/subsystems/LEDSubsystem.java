/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {

  private AddressableLED led = new AddressableLED(Constants.LED_PWM_PORT);
  private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(Constants.LED_COUNT);

  public LEDSubsystem() {
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
    disabled();
  }

  public void disabled() {
    Timer.delay(0.25);
    for (int i = 0; i < ledBuffer.getLength(); i++) { // pattern: solid red
      ledBuffer.setRGB(i, 255, 0, 0);
    }
    led.setData(ledBuffer);
    Timer.delay(0.25);
    for (int i = 0; i < ledBuffer.getLength(); i++) { // pattern: solid red
      ledBuffer.setRGB(i, 0, 0, 0);
    }
    led.setData(ledBuffer);
  }

  public void autoRunning() {
    for (int i = 0; i < ledBuffer.getLength(); i++) { // pattern: solid blue
      ledBuffer.setRGB(i, 0, 0, 255);
    } 
    led.setData(ledBuffer);
  }

  public void shooterAligned() {
    for (int i = 0; i < ledBuffer.getLength(); i++) { // pattern: solid green
      ledBuffer.setRGB(i, 0, 255, 0);
    } 
    led.setData(ledBuffer);
  }

  public void off() {
    for (int i = 0; i < ledBuffer.getLength(); i++) { // pattern: off
      ledBuffer.setRGB(i, 0, 0, 0);
    } 
    led.setData(ledBuffer);
  }

  public void endgameWarning() {
    for (int i = 0; i < ledBuffer.getLength(); i++) { 
      ledBuffer.setRGB(i, 255, 255, 0);
    } 
    led.setData(ledBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DriverStation.getInstance().getMatchTime() == 110) {
      endgameWarning();
    }
  }
}
