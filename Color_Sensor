// TCS3200 Color Detection: Red, Green, Blue, Black

#define S0 9
#define S1 8
#define S2 12
#define S3 13
#define OUT 11

int redFreq, greenFreq, blueFreq;

void setup() {
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);

  // Frequency scaling 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  Serial.begin(9600);
}

void loop() {
  // RED
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  redFreq = pulseIn(OUT, LOW);

  // GREEN
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  greenFreq = pulseIn(OUT, LOW);

  // BLUE
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  blueFreq = pulseIn(OUT, LOW);

  Serial.print("R: "); Serial.print(redFreq);
  Serial.print(" G: "); Serial.print(greenFreq);
  Serial.print(" B: "); Serial.print(blueFreq);

  String color = detectColor(redFreq, greenFreq, blueFreq);
  Serial.print(" => Detected: "); Serial.println(color);

  delay(500);
}


// RED: R: 20-90  G: 90 - 180  B: 70 - 155 

String detectColor(int r, int g, int b) {
  // Lower frequency = higher intensity

  // Black: all very high (little reflected light)
  if (r > 120 && g > 120 && b > 120) return "Black";

  // Red: R strongest (lowest frequency)
  if (r < g && r < b) return "Red";

  // Green: G strongest
  if (g < r && g < b) return "Green";

  // Blue: B strongest
  if (b < r && b < g) return "Blue";

  return "Unknown";
}
