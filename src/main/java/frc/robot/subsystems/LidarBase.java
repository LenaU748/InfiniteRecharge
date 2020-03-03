/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.nio.ByteBuffer;
//import java.util.TimerTask;

import edu.wpi.first.hal.I2CJNI;
//import edu.wpi.first.wpilibj.I2C;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LidarBase extends SubsystemBase {
    private static final byte k_deviceAddress = 0x62;
    private final byte m_port;
    private final ByteBuffer m_buffer = ByteBuffer.allocateDirect(2);
  
    public LidarBase(Port port) {
      m_port = (byte) port.value;
      I2CJNI.i2CInitialize(m_port);
    }
  
    public void startMeasuring() {
      writeRegister(0x04, 0x08 | 32); // default plus bit 5
      writeRegister(0x11, 0xff);
      writeRegister(0x00, 0x04);
    }
  
    public void stopMeasuring() {
      writeRegister(0x11, 0x00);
    }
  
    public double getDistance() {
      return (readShort(0x8f)/2.54) - 6.5;
    }
  
    private int writeRegister(int address, int value) {
      m_buffer.put(0, (byte) address);
      m_buffer.put(1, (byte) value);
  
      return I2CJNI.i2CWrite(m_port, k_deviceAddress, m_buffer, (byte) 2);
    }
  
    private short readShort(int address) {
      m_buffer.put(0, (byte) address);
      I2CJNI.i2CWrite(m_port, k_deviceAddress, m_buffer, (byte) 1);
      I2CJNI.i2CRead(m_port, k_deviceAddress, m_buffer, (byte) 2);
      return m_buffer.getShort(0);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
