package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.LEDPattern;

public class LEDSubsystem extends SubsystemBase 
{
    private static final int kPort = 0;
    private static final int kLength = 10;
  
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_buffer;
  
    public LEDSubsystem() 
    {
      m_led = new AddressableLED(kPort);
      m_buffer = new AddressableLEDBuffer(kLength);
      m_led.setLength(kLength);
      m_led.start();
  
      // Set the default command to turn the strip off, otherwise the last colors written by
      // the last command to run will continue to be displayed.
      // Note: Other default patterns could be used instead!
      setDefaultCommand(runPattern(LEDPattern.solid(Color.kGreen)));
    }
  
    @Override
    public void periodic() 
    {
      // Periodically send the latest LED color data to the LED strip for it to display
      m_led.setData(m_buffer);
    }
  
    /**
     * Creates a command that runs a pattern on the entire LED strip.
     *
     * @param pattern the LED pattern to run
     */
    public Command runPattern(LEDPattern pattern) {
      return run(() -> pattern.applyTo(m_buffer));
    }
  }
