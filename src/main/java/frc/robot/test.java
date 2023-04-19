package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.event.EventLoop;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

public class test implements BooleanSupplier {
    private final EventLoop m_loop;
  private final BooleanSupplier m_condition;

    public test(EventLoop loop, BooleanSupplier condition){
        m_loop = requireNonNullParam(loop, "loop", "Trigger");
    m_condition = requireNonNullParam(condition, "condition", "Trigger");

    }

    public test whileItIsTrue(Command command) {
        requireNonNullParam(command, "command", "whileHigh");
        m_loop.bind(
            new Runnable() {
              private boolean m_pressedLast = m_condition.getAsBoolean();
    
              @Override
              public void run() {
                boolean pressed = m_condition.getAsBoolean();
    
                if (!m_pressedLast && pressed) {
                  command.schedule();
                } else if (m_pressedLast && !pressed) {
                  command.cancel();
                }
    
                m_pressedLast = pressed;
              }
            });
        return this;
      }

      public static DoubleSupplier ConvertButtonToNumber(String m_positiveTrigger, String m_negativeTrigger){
        boolean isPressed;
       boolean isNegPressed;
       System.out.println("hi");

        switch (m_positiveTrigger) {
            case "true":
             isPressed = true;
            
                break;
        
            default:
             isPressed = false;

                break;
        }
        switch (m_negativeTrigger) {
            case "true":
             isNegPressed = true;
            
                break;
        
            default:
             isNegPressed = false;

                break;
        }
         double finalValue;
       if (isNegPressed && !isPressed) {
         finalValue = -1;
       } else if(isPressed && !isNegPressed) {
         finalValue = 1;
       }
       else{
         finalValue = 0;
       }
   
       return () -> finalValue;
     }

    @Override
    public boolean getAsBoolean() {
        // TODO Auto-generated method stub
        return false;
    }
    
}
