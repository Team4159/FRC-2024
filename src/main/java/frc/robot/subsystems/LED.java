package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class LED extends SubsystemBase {
    public static enum LEDState {
        WHITE, BLACK, PURPLE, YELLOW, RED, GREEN, BLUE,
        RAINBOW, RAINBOWCYCLE,
        NONBINARY, GENDERFLUID, GAY, LESBIAN, BI, TRANS
    }

    private final Map<LEDState, Command> commands = new HashMap<LEDState, Command>();
    {
        commands.put(LEDState.WHITE, new InstantCommand(() -> setUniform(75, 75, 75), this));
        commands.put(LEDState.PURPLE, new InstantCommand(() -> setUniform(70, 0, 100), this));
        commands.put(LEDState.YELLOW, new InstantCommand(() -> setUniform(150, 75, 0), this));
        commands.put(LEDState.BLACK, new InstantCommand(() -> setUniform(0, 0, 0), this));
        commands.put(LEDState.RED, new InstantCommand(() -> setUniform(75, 0, 0), this));
        commands.put(LEDState.GREEN, new InstantCommand(() -> setUniform(0, 75, 0), this));
        commands.put(LEDState.BLUE, new InstantCommand(() -> setUniform(0, 0, 75), this));
        commands.put(LEDState.RAINBOW, new ChromaLED((i) -> Color.fromHSV((int)Math.floor(i * 180), 255, 255)).repeatedly());
        commands.put(LEDState.RAINBOWCYCLE, new CycleLinearFlag(new int[]{
            0xE40303, 0xFF8C00, 0xFFED00, 0x008026, 0x004DFF, 0x750787
        }));
        commands.put(LEDState.NONBINARY, new ChromaLinearFlag(new int[]{
            0xFCF434, 0xFCFCFC, 0x9C59D1, 0x2C2C2C
        }).repeatedly());
        commands.put(LEDState.GENDERFLUID, new ChromaLinearFlag(new int[]{
            0xFF75A2, 0xFFFFFF, 0xBE18D6, 0x000000, 0x323DBC
        }).repeatedly());
        commands.put(LEDState.GAY, new ChromaLinearFlag(new int[]{
            0xE40303, 0xFF8C00, 0xFFED00, 0x008026, 0x004DFF, 0x750787
        }).repeatedly());
        commands.put(LEDState.LESBIAN, new ChromaLinearFlag(new int[]{
            0xD52D00, 0xEF7627, 0xFF9A56, 0xFFFFFF, 0xD362A4, 0xB85490, 0xA30262
        }).repeatedly());
        commands.put(LEDState.BI, new ChromaLinearFlag(new int[]{
            0xD60270, 0xD60270, 0x9B4F97, 0x0038A7, 0x0038A7
        }).repeatedly());
        commands.put(LEDState.TRANS, new ChromaLinearFlag(new int[]{
            0x5BCEFA, 0xF5A9B8, 0xFFFFFF, 0xF5A9B8, 0x5BCEFA
        }).repeatedly());
    };
    private final AddressableLED strip;
    private final AddressableLEDBuffer buffer;
    private LEDState state = LEDState.BLACK;
    private boolean mirror;

    public LED(int ledPort, int length, boolean doMirror) {
        strip = new AddressableLED(ledPort);
        buffer = new AddressableLEDBuffer(length);
        strip.setLength(buffer.getLength());
        mirror = doMirror;
    }

    private void setState(LEDState state) {
        if (state.equals(this.state)) return;
        getStateCommand().cancel();
        this.state = state;
        startLED();
    }

    private Command getStateCommand() {
        return commands.get(state);
    }

    private void startLED() {
        strip.start();
        getStateCommand().schedule();
    }

    private void stopLED() {
        getStateCommand().cancel();
        setUniform(0, 0, 0);
        strip.stop();
    }

    private void setUniform(int r, int g, int b) {
        for (int i = 0; i < buffer.getLength(); i++)
            buffer.setRGB(i, r, g, b);
        strip.setData(buffer);
    }

    private void _setWithColor(int i, int color) {
        buffer.setRGB(i,
            (int)Math.floor(((color >> 16)& 255) * Constants.LED.intensityMultiplier),
            (int)Math.floor(((color >> 8) & 255) * Constants.LED.intensityMultiplier), 
            (int)Math.floor(((color >> 0) & 255) * Constants.LED.intensityMultiplier)
        );
    }

    private void setSingle(int i, int color) {
        _setWithColor(i, color);
        if (!mirror) return;
        _setWithColor(buffer.getLength() - 1 - i, color);
    }

    public Command defaultState() {
        return new ChangeState(LEDState.RAINBOW);
    }

    public class ChangeState extends Command {
        private final LEDState desiredState;
        public ChangeState(LEDState state) {
            desiredState = state;
        }

        @Override
        public void initialize() {
            setState(state);
        }
    }

    private class BlinkLED extends SequentialCommandGroup {
        public BlinkLED() {
            super(
                new InstantCommand(() -> LED.this.stopLED()),
                new WaitCommand(Constants.LED.blinkDelay),
                new InstantCommand(() -> LED.this.startLED()),
                new WaitCommand(Constants.LED.blinkDelay)
            );
        }
    }

    private class CycleLED extends Command {
        private CycleColorSupplier supplier;
        private double speedFactor;

        private CycleLED(CycleColorSupplier supplier, double speedMultiplier) {
            this.supplier = supplier;
            this.speedFactor = speedMultiplier;
            addRequirements(LED.this);
        }

        @Override
        public void execute() {
            final int color = supplier.get(System.currentTimeMillis()*(long)speedFactor);
            for (int i = 0; i < buffer.getLength(); i++) setSingle(i, color);
            strip.setData(buffer);
        }

        @FunctionalInterface
        public static interface CycleColorSupplier {
            public int get(long dt);
        }
    }

    private class CycleLinearFlag extends CycleLED {
        private CycleLinearFlag(int[] colors) {
            super((long dt) -> colors[(int)(dt % colors.length)], 1/200d);
        }
    }

    private class ChromaLED extends Command {
        private ChromaColorSupplier supplier;

        private ChromaLED(ChromaColorSupplier supplier) {
            this.supplier = supplier;
            addRequirements(LED.this);
        }

        @Override
        public void execute() {
            final int len = buffer.getLength();
            final int offset = (int)Math.floor((System.currentTimeMillis()/10) % len);
            
            for (int i = 0; i < len; i++) setSingle((i+offset) % len, supplier.get((double)i/len));
            strip.setData(buffer);
        }

        @FunctionalInterface
        public static interface ChromaColorSupplier {
            public int get(double progress);
        }
    }

    private class ChromaLinearFlag extends ChromaLED {
        private ChromaLinearFlag(int[] colors) {
            super((double progress) -> colors[(int)Math.floor(progress*colors.length)]);
        }
    }
}