package frc.robot.subsystems.LEDS;

import java.util.function.Function;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.subsystems.Subsystem;

public class LEDs extends SubsystemBase {
  private static LEDs m_instance;

  private CANdle m_CANdle60;
  private CANdle m_CANdle61;
  private AddressableLED m_led;
  private AddressableLEDBuffer m_buffer;

  private int m_ledTotalLength = Constants.LEDs.k_totalLength;

  // Main sections
  private Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> m_ledStripColor = LEDModes
      .setColor(Color.kRed);

  public static LEDs getInstance() {
    if (m_instance == null) {
      m_instance = new LEDs();
    }
    return m_instance;
  }

  private LEDs() {
    super("LEDs");
    m_CANdle60 = new CANdle(60);
    m_CANdle61 = new CANdle(61);
    m_led = new AddressableLED(Constants.LEDs.k_PWMId);
    m_led.setLength(m_ledTotalLength);
    m_buffer = new AddressableLEDBuffer(m_ledTotalLength);
    m_led.start();
  }

  @Override
  public void periodic() {
    setColorMode();

    m_led.setData(m_buffer);
  }

  public void setColor(Color color) {
    m_CANdle60.setLEDs((int) color.red*255,(int) color.green*255,(int) color.blue*255);
    m_CANdle61.setLEDs((int) color.red*255,(int) color.green*255,(int) color.blue*255);
    m_CANdle60.configBrightnessScalar(1.0);
    m_CANdle61.configBrightnessScalar(1.0);
    m_ledStripColor = LEDModes.setColor(color);
  }

  public void defaultLEDS() {
    breathe();
  }

  public void chase() {
    m_ledStripColor = LEDModes.redChase;
  }

  public void breathe() {
    m_ledStripColor = LEDModes.redBreathe;
  }

  public void rainbow() {
    m_ledStripColor = LEDModes.rainbow;
  }

  public void setColorMode() {
    m_buffer = m_ledStripColor.apply(0).apply(Constants.LEDs.k_totalLength).apply(m_buffer);
  }
}