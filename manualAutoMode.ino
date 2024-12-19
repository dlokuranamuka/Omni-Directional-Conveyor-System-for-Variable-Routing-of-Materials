// Libraries
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>


// constant gobal variables

// Number of cells
const int noCell = 10;

// Radius of a cell
const int cellRadius = 5;

// Define C-space dimensions
const int rows = 300 / 10;
const int cols = 400 / 10;

// The bounds of the object identification zone
const int lowerBoundX = 9;
const int upperBoundX = 19;
const int lowerBoundY = 10;
const int upperBoundY = 15;

// Pin numbering
// Define joystick pins
const int joyX = A0;   // X-axis of the joystick
const int joyY = A1;   // Y-axis of the joystick
const int joySW = 19;  // Joystick button pin (interrupt pin)

// Initial errors of joystick pot readings
const int potErrorX = -9;
const int potErrorY = -15;

// The center value varience for joystick
const int cVary = 30;

// The range width of goal area
const int gVary = 5;

// The range width of object centroid
const int oVary = 2;

// Servo objects
Servo Stear[noCell];

// Lcd object
LiquidCrystal_I2C lcd(0x27, 16, 2);


// Private function prototypes
void controlCell(int cellNo, int cellDir);
void debounceButtonPress();
void manualMode();
void autoMode();
int **initCSpace(int r, int c);
int *initPosition(int x, int y);
void printCSpace(int **space);
void bezierCurve(int xs, int ys, int xg, int yg);
bool isCellOccupied(int cellNo, int x, int y, int radius);
int navigator(int cX, int cY);
void display(char *s1, int p1, char *s2, int p2, int disDelay);
void displayNoDelay(char *s1, int p1, char *s2, int p2);


// Private data types

// Initialize the C-space data structure
// Contains the C-space array and the position of the C-space
typedef struct
{
  int **space;
  int *position;
} cSpace;

// Object data structure
// Contains the radius and the required goal position of the junction
typedef struct
{
  int radius;
  int xGoal;
  int yGoal;
} object;

// Cell data structure
// Contains pin numbers of each cell
typedef struct
{
  int motorPin1;
  int motorPin2;
  int motorPWMPin;
  int servoSignalPin;
} Cell;

// Cell details data structure
// Contains the x and y coordinates of the centroid of the cell
typedef struct
{
  int x;
  int y;
} cellDetail;


// Initialize the C-space 1 as a global variable
cSpace cSpace1;

// Initializing the object types
// large blue
// large green
// small blue
// small green
object objects[4] = {
  { 11, 0, 25 },
  { 11, 29, 25 },
  { 9, 5, 39 },
  { 9, 24, 39 },
};

// Initializing cell pinout using Cell data type
Cell cells[noCell] = {
  { 26, 27, 2, 47 },
  { 28, 29, 3, 48 },
  { 30, 31, 4, 49 },
  { 32, 33, 5, 50 },
  { 34, 35, 6, 51 },
  { 36, 37, 7, 42 },
  { 38, 39, 8, 43 },
  { 40, 41, 9, 44 },
  { 22, 23, 10, 45 },
  { 24, 25, 11, 46 },
};

// Initialize the details of the cells
cellDetail cellDetails[noCell] = {
  { 8, 30 },
  { 15, 35 },
  { 22, 30 },
  { 8, 20 },
  { 15, 25 },
  { 22, 20 },
  { 8, 10 },
  { 15, 15 },
  { 22, 10 },
  { 15, 5 },
};


// Global variables non constant
// PWM signal for the DC motor
int speed = 255;
// Millisecond delay for the servo motors to trun
int dServoTrun = 0;
// Millisecond delay for the cells to work
int dCellWork = 100;
// Interrupt state
volatile bool buttonPressed = true;  // Flag to indicate button press


using namespace std;


void setup() {
  // put your setup code here, to run once:

  // Start serial communication at 9600 baud rate
  Serial.begin(115200);
  while (!Serial) {
    ;  // Wait for the serial port to connect
  }

  // Start i2c communication
  Wire.begin();

  lcd.init();

  // Turning on the back light
  lcd.backlight();

  // Pin mode initializing
  for (int i = 0; i < noCell; i++) {
    pinMode(cells[i].motorPin1, OUTPUT);
    pinMode(cells[i].motorPin2, OUTPUT);
    pinMode(cells[i].motorPWMPin, OUTPUT);
  }

  // Initialize joystick pins
  pinMode(joyX, INPUT);
  pinMode(joyY, INPUT);
  pinMode(joySW, INPUT_PULLUP);

  // Attaching pins for the servos
  for (int i = 0; i < noCell; ++i) {
    Stear[i].attach(cells[i].servoSignalPin);
  }

  // Initialize the C-space1
  cSpace1.space = initCSpace(rows, cols);
  if (cSpace1.space == NULL) {
    // Serial.println("Memory allocation failed for C-space");
    display("Memory alloc", 0, "failed Cspace", 0, 1000);
    return;
  }

  // Initialize the position of the C-space
  cSpace1.position = initPosition(0, 0);
  if (cSpace1.position == NULL) {
    // Serial.println("Memory allocation failed for C-space position");
    display("Memory alloc", 0, "failed pos", 0, 1000);
    return;
  }

  // Attach interrupt to the joystick button (on pin 2)
  attachInterrupt(digitalPinToInterrupt(joySW), debounceButtonPress, FALLING);
}


void loop() {
  // put your main code here, to run repeatedly:

  // Checking if the button is pressed to activate the modes
  if (buttonPressed == false) {
    // Activating auto mode
    displayNoDelay("AUTO", 3, "MODE", 3);
    autoMode();
  } else {
    // Activating manual mode
    displayNoDelay("MANUAL", 2, "MODE", 3);
    manualMode();
  }
}

// Functions

// Function to control individual cell
// Input arguments as cell number and the 2D direction (0 - 35)
void controlCell(int cellNo, int cellDir) {

  // Stopping the DC motor
  digitalWrite(cells[cellNo].motorPin1, 0);
  digitalWrite(cells[cellNo].motorPin2, 0);

  // Getting servo direction
  int servoDir = 0;

  // Checking the angle to convey and conveyig accordingly
  if (cellDir >= 0 && cellDir < 12) {
    servoDir = ((cellDir + 6) * (180 / 17));
    Stear[cellNo].write(servoDir);
    delay(dServoTrun);
    analogWrite(cells[cellNo].motorPWMPin, speed);
    digitalWrite(cells[cellNo].motorPin1, 0);
    digitalWrite(cells[cellNo].motorPin2, 1);
  } else if (cellDir >= 12 && cellDir < 30) {
    servoDir = ((cellDir - 12) * (180 / 17));
    Stear[cellNo].write(servoDir);
    delay(dServoTrun);
    analogWrite(cells[cellNo].motorPWMPin, speed);
    digitalWrite(cells[cellNo].motorPin1, 1);
    digitalWrite(cells[cellNo].motorPin2, 0);
  } else if (cellDir >= 30 && cellDir < 36) {
    servoDir = ((cellDir - 30) * (180 / 17));
    Stear[cellNo].write(servoDir);
    delay(dServoTrun);
    analogWrite(cells[cellNo].motorPWMPin, speed);
    digitalWrite(cells[cellNo].motorPin1, 0);
    digitalWrite(cells[cellNo].motorPin2, 1);
  } else {
    analogWrite(cells[cellNo].motorPWMPin, 0);
    digitalWrite(cells[cellNo].motorPin1, 0);
    digitalWrite(cells[cellNo].motorPin2, 0);
  }
}

// Function for interrupt (switch to manual)
void debounceButtonPress() {
  // Check if debounce time has passed since the last press
  unsigned long currentTime = millis();
  static unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 200;
  if ((currentTime - lastDebounceTime) > debounceDelay) {
    if (buttonPressed == false) {
      buttonPressed = true;
    } else {
      buttonPressed = false;
    }
    lastDebounceTime = currentTime;
  }
}

// Manual mode function
void manualMode() {
  while (buttonPressed == true) {
    // Read analog values
    int rawX = analogRead(joyX) - 512;  // Centering around zero
    int rawY = analogRead(joyY) - 512;  // Centering around zero

    // Getting average value of joystick readings
    int noOfReads = 10;
    int sensSumX = 0;
    int sensSumY = 0;

    for (int i = 0; i < noOfReads; i++) {
      sensSumX += rawX;
      sensSumY += rawY;
    }

    int xValue = -(sensSumX / noOfReads);
    int yValue = (sensSumY / noOfReads);

    // Filtering out the non directed inputs
    if ((xValue <= (potErrorX + cVary) && xValue >= (potErrorX - cVary)) && (yValue <= (potErrorY + cVary) && yValue >= (potErrorY - cVary))) {

      // Deactivating cells
      for (int i = 0; i < noCell; ++i) {
        controlCell(i, -1);
      }

      // Serial.println("not directed");
      // displayNoDelay("not", 0, "directed", 0);
      display("not", 0, "directed", 0, 500);

    } else {

      // Calculate angle in radians and convert to degrees
      float angle = atan2(yValue, xValue) * 180 / PI;

      // Adjust range to 0-360 degrees
      if (angle < 0) {
        angle += 360;
      }

      // Converting angle in to integer in range of 0 to 36
      int angleI = int(angle / 10);

      // Serial.println(angleI);
      displayNoDelay("ANGLE", 3, String(angleI).c_str(), 3);

      // Activating cells
      for (int i = 0; i < noCell; ++i) {
        controlCell(i, angleI);
      }

      // Delay to provide enough time for cells to work
      delay(dCellWork);
    }
  }
}

// Autonomous mode function
void autoMode() {
  // Initialize the C-space with zeros
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      cSpace1.space[i][j] = 0;
    }
  }

  // Deactivating cells
  for (int i = 0; i < noCell; ++i) {
    controlCell(i, -1);
  }

  // Getting serial input
  //Serial.println("automode");
  int objectType;
  int objectX;
  int objectY;

  while (Serial.available() != 3 && buttonPressed == false) {}

  if (Serial.available() >= 3 && buttonPressed == false) {  // Wait for all 3 bytes to be available
    objectType = Serial.read();
    objectX = Serial.read();
    objectY = Serial.read();

    // Send acknowledgment
    Serial.println("ACK");
  }

  // Identifing the object
  if ((objectX > lowerBoundX && objectX < upperBoundX) && (objectY > lowerBoundY && objectY < upperBoundY)) {
    //Serial.println("Object is detected");
    display("Object", 0, "is detected", 0, 200);

    // Checking if the object is in the identification zone
    if (objectType >= 0 && objectType < 4) {
      // Serial.println("Object identified");
      display("Object", 0, "identified", 0, 200);

      // Geting the goal position to generate bezier curve
      int goalx = objects[objectType].xGoal;
      int goaly = objects[objectType].yGoal;

      // Generating bezier curve
      bezierCurve(objectX - 1, objectY - 1, goalx, goaly);

      // Debugging purpose
      //cSpace1.space[objectX - 1][objectY - 1] = 3;
      //printCSpace(cSpace1.space);

      while (buttonPressed == false) {

        // Deactivating cells
        for (int i = 0; i < noCell; ++i) {
          controlCell(i, -1);
        }

        // Getting serial input
        while (Serial.available() != 3 && buttonPressed == false) {}

        if (Serial.available() >= 3) {  // Wait for all 3 bytes to be available
          int demoObjectType = Serial.read();
          objectX = Serial.read();
          objectY = Serial.read();

          // Send acknowledgment
          Serial.println("ACK");
        }

        // If the goal is aquired
        if (((objectX < (goalx + gVary)) && (objectX > (goalx - gVary))) && ((objectY < (goaly + gVary)) && (objectY > (goaly - gVary)))) {

          // Deactivating cells
          for (int i = 0; i < noCell; ++i) {
            controlCell(i, -1);
          }

          // Initialize the C-space with zeros
          for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
              cSpace1.space[i][j] = 0;
            }
          }

          display("object", 0, "navigated", 0, 1000);

          break;
        }

        // If box vanished
        else if ((objectX == 50 && objectY == 60)) {

          // Deactivating cells
          for (int i = 0; i < noCell; ++i) {
            controlCell(i, -1);
          }

          // Initialize the C-space with zeros
          for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
              cSpace1.space[i][j] = 0;
            }
          }

          display("object", 0, "vanished", 0, 1000);

          break;
        }

        else {
        }

        // Finding the direction of conveying
        int dirCon = navigator(objectX - 1, objectY - 1);

        // If path is fully covered
        if (dirCon == -1) {

          // Deactivating cells
          for (int i = 0; i < noCell; ++i) {
            controlCell(i, -1);
          }

          // Initialize the C-space with zeros
          for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
              cSpace1.space[i][j] = 0;
            }
          }

          display("object", 0, "vanished", 0, 1000);

          break;
        }

        // Activating the occupied cells
        for (int k = 0; k < noCell; ++k) {
          if (isCellOccupied(k, objectX, objectY, objects[objectType].radius)) {

            // Activating cell
            controlCell(k, dirCon);

            // Debugging purpose
            // Serial.print("Cell ");
            // Serial.print(k);
            // Serial.println(" is occupied");
          } else {

            // deactivating cell which are not occupied
            controlCell(k, -1);

            // Debugging purpose
            // Serial.print("Cell ");
            // Serial.print(k);
            // Serial.println(" is not occupied");
          }
        }
        // Delay to provide enough time for cells to work
        delay(dCellWork);
      }

    } else {
      displayNoDelay("Object not", 0, "identified", 0);
    }
  } else if (objectY <= lowerBoundY) {
    controlCell(9, 0);
    controlCell(8, 5);
    controlCell(6, 31);
    controlCell(7, 0);
    controlCell(5, 5);
    controlCell(3, 31);
    controlCell(4, 0);
    delay(dCellWork);
  } else {
    display("No Object", 0, "is detected", 0, 1000);
  }
}

// Initialize the C-space
// Cspace is a 2D array of integers that represents the configuration space of size rows x cols
int **initCSpace(int r, int c) {
  // Allocate memory for the C-space
  int **n = (int **)malloc(r * sizeof(int *));
  if (n == NULL) {
    return n;
  }
  for (int i = 0; i < r; i++) {
    n[i] = (int *)malloc(c * sizeof(int));
    if (n[i] == NULL) {
      for (int k = 0; k < i; ++k) {
        // Free allocated rows before returning
        free(n[k]);
      }
      free(n);
      n = NULL;
      return n;
    }
  }

  // Initialize the C-space with zeros
  for (int i = 0; i < r; i++) {
    for (int j = 0; j < c; j++) {
      n[i][j] = 0;
    }
  }

  return n;
}

// Initialize the position of the C-space
// Position is a 1D array of integers that represents the position of the C-space
int *initPosition(int x, int y) {
  int *n = (int *)malloc(2 * sizeof(int));
  if (n == NULL) {
    return n;
  }
  n[0] = x;
  n[1] = y;

  return n;
}

// Print the C-space
// Print the C-space to the console for visualization and debugging
void printCSpace(int **space) {
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      Serial.print(space[i][j]);
    }
    Serial.println();
  }
}

// Bezier curve function
// Compute the Bezier curve for the path planning
// The curve cordinates are set to 2 in the C-space
// 3 control points are used to compute the Bezier curve
// The input arguments are the start and goal positions
// The 2nd control point is set to the center of the C-space
void bezierCurve(int xs, int ys, int xg, int yg) {
  // Control points
  int p0[2] = { xs, ys };
  int p1[2] = { rows / 2, cols / 2 };
  int p2[2] = { xg, yg };

  // Computing the Bezier curve
  // If this takes time reduce the iterations by reducing t increment
  // By reducing iterations the bezier curve will have less points
  for (float t = 0; t <= 1; t += 0.01) {
    int x = (1 - t) * (1 - t) * p0[0] + 2 * (1 - t) * t * p1[0] + t * t * p2[0];
    int y = (1 - t) * (1 - t) * p0[1] + 2 * (1 - t) * t * p1[1] + t * t * p2[1];
    if (x >= 0 && x < rows && y >= 0 && y < cols) {
      // Mark Bezier path in C-space as 2
      cSpace1.space[x][y] = 2;
    }
  }
}

// Cell identifiers
// Identify the cells that are occupied by the object
// The input arguments are the cell number, x and y coordinates, and the radius of the object
// Output is a boolean value that indicates if the cell is occupied
bool isCellOccupied(int cellNo, int x, int y, int radius) {
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      if (pow((i - x), 2) + pow((j - y), 2) <= pow(radius, 2)) {
        if (pow((i - cellDetails[cellNo].x), 2) + pow((j - cellDetails[cellNo].y), 2) <= pow(cellRadius, 2)) {
          return true;
        }
      }
    }
  }
  return false;
}

// Navigator finds the direction which the cells should direct the object
// Input argument are the centroid locations of the object detected
// Returns the angle to direct the object to navigate through the path
int navigator(int cX, int cY) {
  int sX = cX - oVary;
  int sY = cY - oVary;

  // Making the area around centroid zero
  for (int i = 0; i < ((2 * oVary) + 1); i++) {
    if (((sX + i) >= 0) && ((sX + i) < rows)) {
      for (int j = 0; j < ((2 * oVary) + 1); j++) {
        if (((sY + j) >= 0) && ((sY + j) < cols)) {
          cSpace1.space[sX + i][sY + j] = 0;
        }
      }
    }
  }

  // Finding path around centroid and directing the object
  for (int k = oVary + 1; k < 10; k++) {
    int kX = cX - k;
    int kY = cY - k;

    for (int i = 0; i < (2 * k) + 1; i++) {
      if (((kX + i) >= 0) && ((kX + i) < rows)) {
        if (kY >= 0 && kY < cols) {
          if (cSpace1.space[kX + i][kY] == 2) {
            // Calculate angle in radians and convert to degrees
            float angleC = -atan2(((kX + i) - cX), (kY - cY)) * 180 / PI;

            // Adjust range to 0-360 degrees
            if (angleC < 0) {
              angleC += 360;
            }

            // Converting angle in to integer in range of 0 to 36
            int angleCI = int(angleC / 10);
            displayNoDelay("angle", 3, String(angleCI).c_str(), 3);
            return angleCI;
          }
        }
      }
    }

    for (int i = 0; i < (2 * k) + 1; i++) {
      if (((kY + i) >= 0) && ((kY + i) < cols)) {
        if (kX + ((2 * k) + 1) >= 0 && kX + ((2 * k) + 1) < rows) {
          if (cSpace1.space[kX + ((2 * k) + 1)][kY + i] == 2) {
            // Calculate angle in radians and convert to degrees
            float angleC = -atan2((kX + ((2 * k) + 1) - cX), ((kY + i) - cY)) * 180 / PI;

            // Adjust range to 0-360 degrees
            if (angleC < 0) {
              angleC += 360;
            }

            // Converting angle in to integer in range of 0 to 36
            int angleCI = int(angleC / 10);
            displayNoDelay("angle", 3, String(angleCI).c_str(), 3);
            return angleCI;
          }
        }
      }
    }

    for (int i = 0; i < (2 * k) + 1; i++) {
      if (((kX + ((2 * k) + 1) - i) >= 0) && ((kX + ((2 * k) + 1) - i) < rows)) {
        if ((kY + ((2 * k) + 1)) >= 0 && (kY + ((2 * k) + 1)) < cols) {
          if (cSpace1.space[kX + ((2 * k) + 1) - i][kY + ((2 * k) + 1)] == 2) {
            // Calculate angle in radians and convert to degrees
            float angleC = -atan2(((kX + ((2 * k) + 1) - i) - cX), ((kY + ((2 * k) + 1)) - cY)) * 180 / PI;

            // Adjust range to 0-360 degrees
            if (angleC < 0) {
              angleC += 360;
            }

            // Converting angle in to integer in range of 0 to 36
            int angleCI = int(angleC / 10);
            displayNoDelay("angle", 3, String(angleCI).c_str(), 3);
            return angleCI;
          }
        }
      }
    }

    for (int i = 0; i < (2 * k) + 1; i++) {
      if (((kY + ((2 * k) + 1) - i) >= 0) && ((kY + ((2 * k) + 1) - i) < cols)) {
        if (kX >= 0 && kX < rows) {
          if (cSpace1.space[kX][kY + ((2 * k) + 1) - i] == 2) {
            // Calculate angle in radians and convert to degrees
            float angleC = -atan2((kX - cX), ((kY + ((2 * k) + 1) - i) - cY)) * 180 / PI;

            // Adjust range to 0-360 degrees
            if (angleC < 0) {
              angleC += 360;
            }

            // Converting angle in to integer in range of 0 to 36
            int angleCI = int(angleC / 10);
            displayNoDelay("angle", 3, String(angleCI).c_str(), 3);
            return angleCI;
          }
        }
      }
    }
  }

  return -1;
}

// Displaying for debugging purpose and visualization
// Input arguments are the first row string and the first row starting position and the same for the second row and the delay to display
void display(char *s1, int p1, char *s2, int p2, int disDelay) {
  lcd.clear();
  lcd.setCursor(p1, 0);
  lcd.print(s1);
  lcd.setCursor(p2, 1);
  lcd.print(s2);
  delay(disDelay);
  lcd.clear();
}

// Displaying for debugging purpose and visualization countinuously
// Input arguments are the first row string and the first row starting position and the same for the second row
void displayNoDelay(char *s1, int p1, char *s2, int p2) {
  lcd.clear();
  lcd.setCursor(p1, 0);
  lcd.print(s1);
  lcd.setCursor(p2, 1);
  lcd.print(s2);
}