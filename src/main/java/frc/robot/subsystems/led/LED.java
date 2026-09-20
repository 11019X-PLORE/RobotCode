package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTracer;
import org.littletonrobotics.junction.AutoLogOutput;

public class LED extends FullSubsystem {
  public enum LEDState {
    INITIAL,
    SHOOTING,
    BLUE_11019,
    BLUE_SLOW_BLINK,
    TRENCH_WARNING,
    ROTATOR_STALL
  }

  public enum Mode {
    MULTI_SEGMENT,
    SINGLE_COLOR_CYCLE,
    ALL_MARQUEE_LIGHT,
    SHOOTING,
    SOLID,
    SLOW_BLINK,
    RED_BLUE_FAST_BLINK,
    CUSTOM_RGB
  }

  private enum ColorOrder {
    RGB,
    GRB,
    BRG
  }

  private static final ColorOrder kColorOrder = ColorOrder.GRB;
  private static final int kSection1Start = 0;
  private static final int kSection2Start = 5;
  private static final int kSection3Start = 14;
  private static final int kSection4Start = 26;
  private static final int kFireSpeedFrames = 3;
  private static final int kFireShotGapFrames = 12;
  private static final int kSparkShotGapFrames = 8;
  private static final int kChaseSpeedFrames = 2;
  private static final int kAllianceFlameSpeedFrames = 5;
  private static final int kAllianceFlameCount = 8;
  private static final int kAllianceFlameLength = 4;
  private static final double kGreenFireSpeedFrames = 2.5;
  private static final int kGreenFireLength = 2;
  private static final int kGreenFirePerSide = 3;
  private static final int kGreenFireSpacing = 12;
  private static final int kGreenFireCycleSegments = kGreenFirePerSide * kGreenFireSpacing;
  private static final int kFlameCount = 5;
  private static final int kSparkCount = 7;
  private static final int kSlowBlinkFrames = 35;
  private static final int kFastBlinkFrames = 5;
  private static final Color k11019Blue = new Color(18.0 / 90.0, 70.0 / 90.0, 1.0);

  private final AddressableLED led = new AddressableLED(LEDConstants.pwmPort);
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDConstants.ledCount);
  private final int[] renderRed = new int[LEDConstants.ledCount];
  private final int[] renderGreen = new int[LEDConstants.ledCount];
  private final int[] renderBlue = new int[LEDConstants.ledCount];
  private final double[] flamePosition = new double[kFlameCount];
  private final double[] flameSpeed = new double[kFlameCount];
  private final double[] flameLength = new double[kFlameCount];
  private final double[] flamePower = new double[kFlameCount];
  private final boolean[] flameActive = new boolean[kFlameCount];
  private final double[] sparkPosition = new double[kSparkCount];
  private final double[] sparkSpeed = new double[kSparkCount];
  private final double[] sparkPower = new double[kSparkCount];
  private final boolean[] sparkActive = new boolean[kSparkCount];

  @AutoLogOutput private LEDState goal = LEDState.INITIAL;
  private Mode mode = Mode.SOLID;
  private Color modeColor = k11019Blue;
  private boolean hasBeenEnabled;
  private int customRed;
  private int customGreen;
  private int customBlue;
  private int frame;

  public LED() {
    led.setLength(buffer.getLength());
    clear();
    led.setData(buffer);
    led.start();
  }

  public LEDState getGoal() {
    return goal;
  }

  public void setGoal(LEDState goal) {
    this.goal = goal;
  }

  public void setMode(Mode mode) {
    if (this.mode != mode) {
      this.mode = mode;
      resetEffects();
    }
  }

  public Mode getMode() {
    return mode;
  }

  public void multiSegment() {
    setMode(Mode.MULTI_SEGMENT);
  }

  public void SingleColorCycle(Color color) {
    modeColor = color;
    setMode(Mode.SINGLE_COLOR_CYCLE);
  }

  public void allMarqueeLight() {
    setMode(Mode.ALL_MARQUEE_LIGHT);
  }

  public void flameColor(Color color) {
    modeColor = color;
    setMode(Mode.SHOOTING);
  }

  public void SingleColor() {
    solid(k11019Blue);
  }

  public void SingleColor(Color color) {
    solid(color);
  }

  public void slowBlink(Color color) {
    modeColor = color;
    setMode(Mode.SLOW_BLINK);
  }

  public void solid(Color color) {
    modeColor = color;
    setMode(Mode.SOLID);
  }

  public void setCustomRgb(int red, int green, int blue) {
    customRed = clamp(red, 0, LEDConstants.brightness);
    customGreen = clamp(green, 0, LEDConstants.brightness);
    customBlue = clamp(blue, 0, LEDConstants.brightness);
    setMode(Mode.CUSTOM_RGB);
  }

  @Override
  public void periodicAfterScheduler() {
    selectAutomaticMode();
    runSelectedMode();
    LoggedTracer.record("LED/OutputData");
  }

  private void selectAutomaticMode() {
    if (DriverStation.isEnabled()) {
      hasBeenEnabled = true;
    }

    if (mode == Mode.CUSTOM_RGB) {
      return;
    }

    if (DriverStation.isDisabled()) {
      if (hasBeenEnabled) {
        multiSegment();
      } else if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
          == DriverStation.Alliance.Red) {
        SingleColorCycle(Color.kRed);
      } else {
        SingleColorCycle(Color.kBlue);
      }
    } else if (DriverStation.isAutonomous()) {
      allMarqueeLight();
    } else if (goal == LEDState.TRENCH_WARNING) {
      SingleColorCycle(Color.kYellow);
    } else if (goal == LEDState.ROTATOR_STALL) {
      setMode(Mode.RED_BLUE_FAST_BLINK);
    } else if (goal == LEDState.SHOOTING) {
      flameColor(Color.kGreen);
    } else if (goal == LEDState.BLUE_SLOW_BLINK) {
      slowBlink(Color.kBlue);
    } else {
      SingleColor();
    }
  }

  private void runSelectedMode() {
    switch (mode) {
      case MULTI_SEGMENT -> runMultiSegment();
      case SINGLE_COLOR_CYCLE -> runSingleColorCycle();
      case ALL_MARQUEE_LIGHT -> runAllMarqueeLight();
      case SHOOTING -> runShooting();
      case SOLID -> runSolidColor();
      case SLOW_BLINK -> runSlowBlink();
      case RED_BLUE_FAST_BLINK -> runRedBlueFastBlink();
      case CUSTOM_RGB -> runCustomRgb();
    }
  }

  private void runMultiSegment() {
    clearRenderBuffer();
    runReverseFire(kSection2Start, kSection3Start - 1);
    runReverseSparks(kSection1Start, kSection2Start - 1);
    runRainbowChase(kSection3Start, kSection4Start - 1, 0);
    runRainbowChase(kSection4Start, LEDConstants.ledCount - 1, 55);
    showFrame();
  }

  private void runSingleColorCycle() {
    clearRenderBuffer();
    int offset = (frame / kAllianceFlameSpeedFrames) % LEDConstants.ledCount;
    for (int flame = 0; flame < kAllianceFlameCount; flame++) {
      int head =
          wrapLed(
              offset
                  + (int) Math.round(flame * (double) LEDConstants.ledCount / kAllianceFlameCount));
      drawSingleColorCycle(head, modeColor);
    }
    showFrame();
  }

  private void runAllMarqueeLight() {
    clearRenderBuffer();
    runRainbowChase(0, LEDConstants.ledCount - 1, 0);
    showFrame();
  }

  private void runShooting() {
    clearRenderBuffer();
    int centerLeft = (kSection3Start + kSection4Start - 1) / 2;
    int centerRight = centerLeft + 1;
    double travel = (frame / kGreenFireSpeedFrames) % kGreenFireCycleSegments;
    for (int i = 0; i < kGreenFirePerSide; i++) {
      double distance = (travel + (i * kGreenFireSpacing)) % kGreenFireCycleSegments;
      drawShootingFlame((int) Math.round(centerLeft - distance), -1, modeColor);
      drawShootingFlame((int) Math.round(centerRight + distance), 1, modeColor);
    }
    showFrame();
  }

  private void runSolidColor() {
    clearRenderBuffer();
    for (int i = 0; i < LEDConstants.ledCount; i++) {
      addRenderColor(i, colorRed(modeColor), colorGreen(modeColor), colorBlue(modeColor));
    }
    showFrame();
  }

  private void runSlowBlink() {
    clearRenderBuffer();
    double phase = (frame % kSlowBlinkFrames) / (double) kSlowBlinkFrames;
    double wave = 0.28 + (0.72 * ((Math.sin(phase * 2.0 * Math.PI) + 1.0) / 2.0));
    int value = (int) Math.round(LEDConstants.brightness * wave);

    for (int i = 0; i < LEDConstants.ledCount; i++) {
      addRenderColor(
          i,
          scaledColorRed(modeColor, value),
          scaledColorGreen(modeColor, value),
          scaledColorBlue(modeColor, value));
    }

    showFrame();
  }

  private void runRedBlueFastBlink() {
    clearRenderBuffer();
    boolean redFrame = (frame / kFastBlinkFrames) % 2 == 0;
    int red = redFrame ? LEDConstants.brightness : 0;
    int blue = redFrame ? 0 : LEDConstants.brightness;
    for (int i = 0; i < LEDConstants.ledCount; i++) {
      addRenderColor(i, red, 0, blue);
    }
    showFrame();
  }

  private void runCustomRgb() {
    clearRenderBuffer();
    for (int i = 0; i < LEDConstants.ledCount; i++) {
      addRenderColor(i, customRed, customGreen, customBlue);
    }
    showFrame();
  }

  private void drawShootingFlame(int head, int direction, Color color) {
    int[] levels = {LEDConstants.brightness, 58};
    for (int i = 0; i < kGreenFireLength; i++) {
      int index = head - (direction * i);
      if (index >= 0 && index < LEDConstants.ledCount) {
        addRenderColor(
            index,
            scaledColorRed(color, levels[i]),
            scaledColorGreen(color, levels[i]),
            scaledColorBlue(color, levels[i]));
      }
    }
  }

  private void drawSingleColorCycle(int head, Color color) {
    int[] levels = {LEDConstants.brightness, 80, 68, 56};
    for (int i = 0; i < kAllianceFlameLength; i++) {
      int index = wrapLed(head - i);
      addRenderColor(
          index,
          scaledColorRed(color, levels[i]),
          scaledColorGreen(color, levels[i]),
          scaledColorBlue(color, levels[i]));
    }
  }

  private void runReverseFire(int start, int end) {
    if (frame % kFireShotGapFrames == 0) {
      shootReverseFlame(end);
    }
    if (frame % kFireSpeedFrames == 0) {
      moveReverseFlames(start);
    }
    for (int i = 0; i < kFlameCount; i++) {
      if (flameActive[i]) {
        drawReverseFlame(i, start, end);
      }
    }
  }

  private void shootReverseFlame(int end) {
    for (int i = 0; i < kFlameCount; i++) {
      if (!flameActive[i]) {
        flameActive[i] = true;
        flamePosition[i] = end;
        flameSpeed[i] = randomDouble(0.35, 0.52);
        flameLength[i] = randomDouble(2.0, 3.4);
        flamePower[i] = randomDouble(0.82, 1.0);
        return;
      }
    }
  }

  private void moveReverseFlames(int start) {
    for (int i = 0; i < kFlameCount; i++) {
      if (flameActive[i]) {
        flamePosition[i] -= flameSpeed[i];
        if (flamePosition[i] + flameLength[i] < start) {
          flameActive[i] = false;
        }
      }
    }
  }

  private void drawReverseFlame(int flame, int start, int end) {
    int drawStart = Math.max(start, (int) Math.floor(flamePosition[flame] - 1.0));
    int drawEnd = Math.min(end, (int) Math.ceil(flamePosition[flame] + flameLength[flame]));
    for (int led = drawStart; led <= drawEnd; led++) {
      double behind = led - flamePosition[flame];
      if (behind < -1.0 || behind > flameLength[flame]) {
        continue;
      }
      double section = Math.max(0.0, behind / flameLength[flame]);
      double intensity;
      int red;
      int green;
      if (behind < 0.0) {
        intensity = (1.0 + behind) * 0.32;
        red = 90;
        green = 0;
      } else if (section < 0.22) {
        double t = section / 0.22;
        intensity = 1.0 - (0.10 * t);
        red = 255;
        green = 36 - (int) (14 * t);
      } else if (section < 0.62) {
        double t = (section - 0.22) / 0.40;
        intensity = 0.86 - (0.36 * t);
        red = 220 - (int) (55 * t);
        green = 18 - (int) (15 * t);
      } else {
        double t = (section - 0.62) / 0.38;
        intensity = 0.40 * Math.pow(1.0 - t, 1.8);
        red = 120;
        green = 0;
      }
      addRenderColor(
          led,
          (int) (red * intensity * flamePower[flame] * LEDConstants.brightness / 255.0),
          (int) (green * intensity * flamePower[flame] * LEDConstants.brightness / 255.0),
          0);
    }
  }

  private void runReverseSparks(int start, int end) {
    if (frame % kSparkShotGapFrames == 0) {
      shootReverseSpark(end);
    }
    if (frame % kFireSpeedFrames == 0) {
      moveReverseSparks(start);
    }
    for (int i = 0; i < kSparkCount; i++) {
      if (!sparkActive[i]) {
        continue;
      }
      int led = clamp((int) Math.round(sparkPosition[i]), start, end);
      addRenderColor(led, (int) (LEDConstants.brightness * sparkPower[i]), 2, 0);
      int tail = led + 1;
      if (tail <= end) {
        addRenderColor(tail, (int) (LEDConstants.brightness * 0.28 * sparkPower[i]), 0, 0);
      }
    }
  }

  private void shootReverseSpark(int end) {
    for (int i = 0; i < kSparkCount; i++) {
      if (!sparkActive[i]) {
        sparkActive[i] = true;
        sparkPosition[i] = end;
        sparkSpeed[i] = randomDouble(0.20, 0.45);
        sparkPower[i] = randomDouble(0.25, 0.70);
        return;
      }
    }
  }

  private void moveReverseSparks(int start) {
    for (int i = 0; i < kSparkCount; i++) {
      if (sparkActive[i]) {
        sparkPosition[i] -= sparkSpeed[i];
        sparkPower[i] *= 0.82;
        if (sparkPosition[i] < start || sparkPower[i] < 0.05) {
          sparkActive[i] = false;
        }
      }
    }
  }

  private void runRainbowChase(int start, int end, int hueOffset) {
    int length = end - start + 1;
    int offset = (frame / kChaseSpeedFrames) % Math.max(1, length);
    for (int led = start; led <= end; led++) {
      int local = led - start;
      int hue = hueOffset + (((local + offset) * 180) / Math.max(1, length));
      int wave = ((local + offset) % 6 == 0) ? LEDConstants.brightness : 35;
      setRenderHue(led, hue, wave);
    }
  }

  private void setRenderHue(int ledIndex, int hue, int value) {
    hue = ((hue % 180) + 180) % 180;
    int region = hue / 30;
    int remainder = (hue % 30) * 255 / 30;
    int red;
    int green;
    int blue;
    switch (region) {
      case 0 -> {
        red = 255;
        green = remainder;
        blue = 0;
      }
      case 1 -> {
        red = 255 - remainder;
        green = 255;
        blue = 0;
      }
      case 2 -> {
        red = 0;
        green = 255;
        blue = remainder;
      }
      case 3 -> {
        red = 0;
        green = 255 - remainder;
        blue = 255;
      }
      case 4 -> {
        red = remainder;
        green = 0;
        blue = 255;
      }
      default -> {
        red = 255;
        green = 0;
        blue = 255 - remainder;
      }
    }
    addRenderColor(ledIndex, red * value / 255, green * value / 255, blue * value / 255);
  }

  private void resetEffects() {
    frame = 0;
    for (int i = 0; i < kFlameCount; i++) {
      flameActive[i] = false;
    }
    for (int i = 0; i < kSparkCount; i++) {
      sparkActive[i] = false;
    }
  }

  private void showFrame() {
    flushRenderBuffer();
    frame++;
    led.setData(buffer);
  }

  private void clear() {
    for (int i = 0; i < LEDConstants.ledCount; i++) {
      setLedColor(i, 0, 0, 0);
    }
  }

  private void clearRenderBuffer() {
    for (int i = 0; i < LEDConstants.ledCount; i++) {
      renderRed[i] = 0;
      renderGreen[i] = 0;
      renderBlue[i] = 0;
    }
  }

  private void addRenderColor(int ledIndex, int red, int green, int blue) {
    renderRed[ledIndex] = Math.max(renderRed[ledIndex], clamp(red, 0, LEDConstants.brightness));
    renderGreen[ledIndex] =
        Math.max(renderGreen[ledIndex], clamp(green, 0, LEDConstants.brightness));
    renderBlue[ledIndex] = Math.max(renderBlue[ledIndex], clamp(blue, 0, LEDConstants.brightness));
  }

  private void flushRenderBuffer() {
    for (int i = 0; i < LEDConstants.ledCount; i++) {
      setLedColor(i, renderRed[i], renderGreen[i], renderBlue[i]);
    }
  }

  private int colorRed(Color color) {
    return scaledColorRed(color, LEDConstants.brightness);
  }

  private int colorGreen(Color color) {
    return scaledColorGreen(color, LEDConstants.brightness);
  }

  private int colorBlue(Color color) {
    return scaledColorBlue(color, LEDConstants.brightness);
  }

  private int scaledColorRed(Color color, int value) {
    return clamp((int) Math.round(color.red * value), 0, LEDConstants.brightness);
  }

  private int scaledColorGreen(Color color, int value) {
    return clamp((int) Math.round(color.green * value), 0, LEDConstants.brightness);
  }

  private int scaledColorBlue(Color color, int value) {
    return clamp((int) Math.round(color.blue * value), 0, LEDConstants.brightness);
  }

  private void setLedColor(int ledIndex, int red, int green, int blue) {
    switch (kColorOrder) {
      case GRB -> buffer.setRGB(ledIndex, green, red, blue);
      case BRG -> buffer.setRGB(ledIndex, blue, red, green);
      case RGB -> buffer.setRGB(ledIndex, red, green, blue);
    }
  }

  private int wrapLed(int ledIndex) {
    return ((ledIndex % LEDConstants.ledCount) + LEDConstants.ledCount) % LEDConstants.ledCount;
  }

  private double randomDouble(double minInclusive, double maxExclusive) {
    return minInclusive + (Math.random() * (maxExclusive - minInclusive));
  }

  private int clamp(int value, int min, int max) {
    return Math.max(min, Math.min(max, value));
  }
}
