package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.event.EventLoop;

import java.util.function.BooleanSupplier;

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

    @Override
    public boolean getAsBoolean() {
        // TODO Auto-generated method stub
        return false;
    }
    
}
