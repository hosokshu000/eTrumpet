int valve1 = 2;
int valve2 = 4;
int valve3 = 7;
int piezo = 8;
int pot = A0;

int tones[] = {233, 349, 466, 587, 698, 932};
int valves[] = {0, 0, 0};
int valvePins[] = {valve1, valve2, valve3};

void setup() {
  pinMode(valve1, INPUT);
  pinMode(valve2, INPUT);
  pinMode(valve3, INPUT);
  pinMode(piezo, OUTPUT);
  pinMode(pot, INPUT);
}

void loop() {
  bool isPlaying = false;
  int potVal = analogRead(pot); // Raw analog input value

  // Simulate when no air is passing through
  if (potVal > 10) {
    isPlaying = true;
  }

  potVal -= 10; // Subtract 10 as potVal between 0 - 10 means there's no air passing through

  int partial = tones[potVal / 169]; // Convert analog input to partial frequency by evenly dividing potVal into 6 different intervals representing each partial

  // Populate valves array: 1 - valve pressed, 0 - valve not pressed
  for(int i = 0; i < 3; i++) {
    if (digitalRead(valvePins[i]) == HIGH) {
      valves[i] = 1;
    }
    else {
      valves[i] = 0;
    }
  }

  int numSemitone = -(2 * valves[0] + valves[1] + 3 * valves[2]); // Number of semitones to go down from partial
  int pitch = (int)(partial * pow(2, numSemitone / 12.0)); // Output frequency, equal temperament system

  // Only play if isPlaying is true
  if (isPlaying) {
    tone(piezo, pitch);
  }
  else {
    noTone(piezo);
  }
}