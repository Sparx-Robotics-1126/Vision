package frc.team1126.commands.LED;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.subsystems.LEDs;

public class RainbowCommand extends Command {
    private final LEDs ledSubsystem;

    public RainbowCommand(LEDs ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
        addRequirements(ledSubsystem);
    }

    @Override
    public void execute() {
        ledSubsystem.setRainbow();
    }

    @Override
    public boolean isFinished() {
        return false; // Run until interrupted
    }
}
