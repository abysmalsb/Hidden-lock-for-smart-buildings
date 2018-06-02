/******************************************************************************
   This project is made for the "Sensing the World in All Three Dimensions"
   contest on www.hackster.io.

   See the full project description here:
   https://www.hackster.io/Abysmal/hidden-door-lock-for-smart-buildings-b17bb5

   author: Balázs Simon
   e-mail: simon.balazs.1992@gmail.com

*******************************************************************************/

#include <Tle493d_w2b6.h>

#define SOLENOID_PIN 8
#define MAGNET_TRIGGER_LEVEL 1.0f             // Used for nearby magnet detection
#define DECODER_STEPS 8                       // The program will differentiate 8 different states based on 
                                              // the rotation of the magnet
#define ANGLE_STEP 2 * PI / DECODER_STEPS     // How big is one step in angles
#define SMOOTHING ANGLE_STEP / 4              // Used for solving the bouncing issue
#define LEAVE_THE_LOCK_UNLOCKED_MILLIS 10000  // The lock will stay unlocked for 10 seconds

Tle493d_w2b6 magnetic3DSensor = Tle493d_w2b6();

int lockCombination[] = {2, 5, 1, 7};
int inputCombination[] = {0, 0, 0, 0};
int combinationLength = 4;
int lastEntryIndex = 0;

long unlockedTheLockAt = 0;
bool lockUnlocked = false;
bool rotatingClockwise = true;
bool newLockCode = false;
float lastValidAngle = 0.0f;

void setup() {
  Serial.begin(9600);
  // LED_BULTIN used as a lock state indicator
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SOLENOID_PIN, OUTPUT);

  magnetic3DSensor.begin();
  magnetic3DSensor.begin();
  magnetic3DSensor.enableTemp();
}

void loop() {
  magnetic3DSensor.updateData();
  if (magnetic3DSensor.getNorm() > MAGNET_TRIGGER_LEVEL) {
    processAngle(magnetic3DSensor.getAzimuth());

    if (newLockCode) {
      newLockCode = false;
      if (checkIfCombinationCorrect()) {
        unlockLock();
      }
    }
  }

  if (lockUnlocked && millis() - unlockedTheLockAt > LEAVE_THE_LOCK_UNLOCKED_MILLIS) {
    closeLock();
  }
}

/**
 * Unlocking the lock
 */
void unlockLock() {
  lockUnlocked = true;
  unlockedTheLockAt = millis();
  Serial.println("Lock: unlocked");
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(SOLENOID_PIN, HIGH);
}

/**
 * Closing the lock
 */
void closeLock() {
  lockUnlocked = false;
  Serial.println("Lock: closed");
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(SOLENOID_PIN, LOW);
}

/**
 * Filtering the angles to prevent bouncing and saving the new input code when it is needed
 */
void processAngle(float angle) {
  if (rotatingClockwise) {
    if ((lastValidAngle < angle && lastValidAngle + PI > angle) || lastValidAngle - PI > angle) {
      lastValidAngle = angle;
    }
    else if ((lastValidAngle > angle + SMOOTHING && lastValidAngle + PI > angle)
             || (lastValidAngle + 2 * PI > angle + SMOOTHING && angle - lastValidAngle > PI)) {
      lastValidAngle = angle;
      int code = decodeAngleToPosition(angle);
      registerNewCode(code);
      newLockCode = true;
      rotatingClockwise = false;
    }
  }
  else {
    if ((lastValidAngle > angle && lastValidAngle - PI < angle) || lastValidAngle + PI < angle) {
      lastValidAngle = angle;
    }
    else if ((lastValidAngle < angle - SMOOTHING  && lastValidAngle - PI < angle)
             || (lastValidAngle - 2 * PI < angle - SMOOTHING && lastValidAngle - angle > PI)) {
      lastValidAngle = angle;
      int code = decodeAngleToPosition(angle);
      registerNewCode(code);
      newLockCode = true;
      rotatingClockwise = true;
    }
  }
}

/**
 * This function will assign an integer state value to the given angle, It can be 1-8 in the current implementation
 */
int decodeAngleToPosition(float angle) {
  for (int i = 1; i <= DECODER_STEPS; i++) {
    if (angle <= - PI + i * ANGLE_STEP + 0.0001)
      return i;
  }
  return -1;
}

void registerNewCode(int code) {
  if (lastEntryIndex == combinationLength)
    lastEntryIndex = 0;

  inputCombination[lastEntryIndex] = code;
  lastEntryIndex++;
}

/**
 * Checking whether the currently saved input combination is the correct combination
 */
bool checkIfCombinationCorrect() {
  bool result = true;
  Serial.print("Input combination: ");
  for (int i = lastEntryIndex, j = 0; j < combinationLength; i++, j++) {
    if (i == combinationLength)
      i = 0;

    Serial.print(inputCombination[i]);

    if (inputCombination[i] != lockCombination[j]) {
      result = false;
    }
  }
  Serial.println();
  return result;
}

