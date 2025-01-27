package frc.team1126.commands.LED;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.subsystems.LEDs;

public class PulseCommand extends Command {
    private final LEDs ledSubsystem;
    private final Color8Bit color;
    private final int pulseRate;

    public PulseCommand(LEDs ledSubsystem, Color8Bit color, int pulseRate) {
        this.ledSubsystem = ledSubsystem;
        this.color = color;
        this.pulseRate = pulseRate;
        addRequirements(ledSubsystem);
    }

    @Override
    public void execute() {
        ledSubsystem.setPulse(color, pulseRate);
    }

    @Override
    public boolean isFinished() {
        return false; // Run until interrupted
    }
}
