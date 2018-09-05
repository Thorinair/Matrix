#include "Adafruit_NeoPixel.h"
#ifdef __AVR__
  #include "avr/power.h"
#endif

#include <EEPROM.h>

// Pins
#define PIN_BUTTON_SENSITIVITY 4
#define PIN_BUTTON_BRIGHTNESS  5
#define PIN_BUTTON_MODE        6
#define PIN_BUTTON_COLOR       7

#define PIN_MIC_INPUT A0
#define PIN_MIC_GAIN  8
#define PIN_MIC_AR    9

#define PIN_PIXEL     10


// FHT Setup
#define OCT_NORM 0 // 0: no normalisation, more high freq 1: divided by number of bins, less high freq
#define OCTAVE 1 // use the octave output function
#define FHT_N 128 // set to 128 point fht

#include <FHT.h> // include the library

// EEPROM
#define EEPROM_RESET         false
#define EEPROM_SAVED_ADDRESS 0
#define EEPROM_SAVED_VALUE   1

// Settings
#define MODE_CNT 1
#define MODE_DEF 0
#define MODE_EPR 1
uint8_t mode = MODE_DEF;
bool    modePress = false;

#define BRIGHTNESS_MIN 10
#define BRIGHTNESS_MAX 120
#define BRIGHTNESS_STP 10
#define BRIGHTNESS_DEF 60
#define BRIGHTNESS_EPR 2
uint8_t brightness = BRIGHTNESS_DEF;
bool    brightnessPress = false;

#define SENSITIVITY_MIN 1
#define SENSITIVITY_MAX 10
#define SENSITIVITY_DEF 7
#define SENSITIVITY_EPR 3
uint8_t sensitivity = SENSITIVITY_DEF;
bool    sensitivityPress = false;

#define COLOR_DEF 0
#define COLOR_EPR 4
uint8_t colorCounts[MODE_CNT] = {15};
uint8_t color[MODE_CNT] = {COLOR_DEF};
bool    colorPress = false;


// Matrix
#define MATRIX_ROWS   10
#define MATRIX_COLS   10

Adafruit_NeoPixel strip = Adafruit_NeoPixel(100, PIN_PIXEL, NEO_GRB + NEO_KHZ800);
uint32_t matrix[MATRIX_ROWS][MATRIX_COLS] = {0};


// Bins
#define BINS_SRC  7
#define BINS_DEST 10

int32_t oldBins[BINS_SRC] = {0};
int8_t  bins[BINS_DEST];


// Noise debug
#define PROFILE_NOISES false
#define NOISE_WAIT     500
uint16_t noises[BINS_SRC] = {0};
uint16_t delayProfile = NOISE_WAIT;


// Info bar
#define INFO_BAR_FADE 200
uint16_t infoBarFade = 0;
uint8_t  infoBarValue;


// Multipliers
#define MULTI_MULTIS  10
//#define MULTI_MAP     45
#define MULTI_MAP     9
//#define MULTI_MAP     3
#define MULTI_POWER   10

#define DECAY         3

uint16_t binNoises[] = {211, 195, 87, 86, 90, 90, 91};
uint16_t binMultis[] = {30,  30,  4,  5,  5,  5,  50};


// Map
struct structBinMap {
    //int ratios[BINS_DEST][2] = {{45, 0}, {25, 20}, {5, 40}, {30, 15}, {10, 35}, {35, 10}, {15, 30}, {40, 5}, {20, 25}, {0, 45}};
    //int bins[BINS_DEST][2]   = {{1,  2}, {1,  2 }, {1, 2 }, {2,  3 }, {2,  3 }, {3,  4 }, {3,  4 }, {4,  5}, {4,  5 }, {4, 5 }};
    uint8_t ratios[BINS_DEST][2] = {{9, 0}, {4, 5}, {8, 1}, {3, 6}, {7, 2}, {2, 7}, {6, 3}, {1, 8}, {5, 4}, {0, 9}};
    uint8_t bins[BINS_DEST][2]   = {{0, 1}, {0, 1}, {1, 2}, {1, 2}, {2, 3}, {2, 3}, {3, 4}, {3, 4}, {4, 5}, {4, 5}};
    //int ratios[BINS_DEST][2] = {{3, 0}, {1, 2}, {2, 1}, {0, 3}, {1, 2}, {2, 1}, {3, 0}, {1, 2}, {2, 1}, {0, 3}};
    //int bins[BINS_DEST][2]   = {{0, 1}, {0, 1}, {1, 2}, {1, 2}, {2, 3}, {3, 4}, {4, 5}, {4, 5}, {5, 6}, {5, 6}};
} binMap;


// Setup
void setupSettings();
void setupPins();
void setupMicrophone();
void setupRegisters();

// FHT and processing
void processFHT();
void processBins();
void processButtons();

// Visualiser drawing
void drawVisualiser();
void drawInfoBar();
void drawNoiseDebug();
void drawMode_EQPro();

// Matrix pushing
void pushMatrix();



// Setup
void setupSettings() {
    if (EEPROM.read(EEPROM_SAVED_ADDRESS) != EEPROM_SAVED_VALUE || EEPROM_RESET) {
        EEPROM.write(EEPROM_SAVED_ADDRESS, EEPROM_SAVED_VALUE);
        
        EEPROM.write(MODE_EPR, mode);
        EEPROM.write(BRIGHTNESS_EPR, brightness);
        EEPROM.write(SENSITIVITY_EPR, sensitivity);
        for (uint8_t i = 0; i < MODE_CNT; i++)
            EEPROM.write(COLOR_EPR + i, color[i]);
    }
    else {
        mode = EEPROM.read(MODE_EPR);
        brightness = EEPROM.read(BRIGHTNESS_EPR);
        sensitivity = EEPROM.read(SENSITIVITY_EPR);
        for (uint8_t i = 0; i < MODE_CNT; i++)
            color[i] = EEPROM.read(COLOR_EPR + i);
    }
}

void setupPins() {
    pinMode(PIN_BUTTON_SENSITIVITY, INPUT);
    pinMode(PIN_BUTTON_BRIGHTNESS, INPUT);
    pinMode(PIN_BUTTON_MODE, INPUT);
    pinMode(PIN_BUTTON_COLOR, INPUT);
}

void setupMicrophone() {      
    // Set Gain LOW 50dB, HIGH 40 dB, INPUT 60dB
    //pinMode(PIN_MIC_GAIN, INPUT);
    digitalWrite(PIN_MIC_GAIN, HIGH);

    // Set Attack and Release Ratios
    // LOW 1:500, HIGH 1:2000, INPUT 1:4000
    pinMode(PIN_MIC_AR, INPUT);
    //digitalWrite(PIN_MIC_AR, LOW);
}

void setupRegisters() {
    TIMSK0 = 0; // turn off timer0 for lower jitter
    ADCSRA = 0xe5; // set the adc to free running mode
    ADMUX = 0x40; // use adc0
    DIDR0 = 0x01; // turn off the digital input for adc0  
}


// FHT and processing
void processFHT() {
    // Signal capture and FHT
    cli();  // UDRE interrupt slows this way down on arduino1.0
    for (uint8_t n = 0 ; n < FHT_N ; n++) { // save 128 samples
        while(!(ADCSRA & 0x10)); // wait for adc to be ready
        ADCSRA = 0xf5; // restart adc
        byte l = ADCL; // fetch adc data
        byte h = ADCH;
        int k = (h << 8) | l; // form into an int
        k -= 0x0200; // form into a signed int
        k <<= 6; // form into a 16b signed int
        fht_input[n] = k; // put real data into bins
    }
    fht_window(); // window the data for better frequency response
    fht_reorder(); // reorder the data before doing the fht
    fht_run(); // process the data in the fht
    fht_mag_octave(); // take the output of the fht
    sei();    
}

void processBins() {
    uint8_t i;

    int32_t decay = MULTI_MULTIS * (SENSITIVITY_MAX + 1 - sensitivity) / DECAY;    
    float multi = (float) 1 / MULTI_MAP / MULTI_MULTIS / (SENSITIVITY_MAX + 1 - sensitivity);
    
    // Noise cleanup, scaling and decay calculations
    for (i = 0; i < BINS_SRC; i++) {
        int val = max((fht_oct_out[i] - binNoises[i]) * binMultis[i], 0);
        if (oldBins[i] > val)
            oldBins[i] -= decay;
        else
            oldBins[i] = val;
    }

    // Linear upscaling to 10 bins
    for (i = 0; i < BINS_DEST; i++) {
        bins[i] = ((float)oldBins[binMap.bins[i][0]] * binMap.ratios[i][0] + oldBins[binMap.bins[i][1]] * binMap.ratios[i][1]) * multi;
    }
}

void processButtons() {
    if (digitalRead(PIN_BUTTON_COLOR) == HIGH && !colorPress) {
        colorPress = true;
        if (color[mode] >= colorCounts[mode] - 1)
            color[mode] = 0;
        else
            color[mode]++;
        EEPROM.write(COLOR_EPR + mode, color[mode]);    
    }
    else if (digitalRead(PIN_BUTTON_COLOR) == LOW && colorPress) {
        colorPress = false;  
    }

    if (digitalRead(PIN_BUTTON_MODE) == HIGH && !modePress) {
        modePress = true;
        if (mode >= MODE_CNT - 1)
            mode = 0;
        else
            mode++;
        EEPROM.write(MODE_EPR, mode);    
    }
    else if (digitalRead(PIN_BUTTON_MODE) == LOW && modePress) {
        modePress = false;  
    }
    
    if (digitalRead(PIN_BUTTON_BRIGHTNESS) == HIGH && !brightnessPress) {
        brightnessPress = true;
        if (brightness >= BRIGHTNESS_MAX)
            brightness = BRIGHTNESS_MIN;
        else
            brightness += BRIGHTNESS_STP; 
        EEPROM.write(BRIGHTNESS_EPR, brightness);   
    }
    else if (digitalRead(PIN_BUTTON_BRIGHTNESS) == LOW && brightnessPress) {
        brightnessPress = false;  
    }
    
    if (digitalRead(PIN_BUTTON_SENSITIVITY) == HIGH && !sensitivityPress) {
        sensitivityPress = true;
        if (infoBarFade > 0) {
            if (sensitivity >= SENSITIVITY_MAX)
                sensitivity = SENSITIVITY_MIN;
            else
                sensitivity++;
            EEPROM.write(SENSITIVITY_EPR, sensitivity);
        }
        infoBarFade = INFO_BAR_FADE;
    }
    else if (digitalRead(PIN_BUTTON_SENSITIVITY) == LOW && sensitivityPress) {
        sensitivityPress = false;  
    }
}



// Visualiser drawing
void drawVisualiser() {
    switch (mode) {
        // MODE 0: EQPro
        // Equalizer-like horizontal visualisation with lowest frequencies being on the left.
        case 0: drawMode_EQPro(); break;
    }
    
    drawInfoBar();
}

void drawInfoBar() {
    if (infoBarFade > 0) {
        uint8_t i;
        for (i = SENSITIVITY_MIN; i <= SENSITIVITY_MAX; i++) {
            if (i <= sensitivity)
                matrix[MATRIX_ROWS - 1][MATRIX_COLS - 1 + SENSITIVITY_MIN - i] = strip.Color(brightness, brightness, brightness);
            else
                matrix[MATRIX_ROWS - 1][MATRIX_COLS - 1 + SENSITIVITY_MIN - i] = strip.Color(0, 0, 0);
        } 
        infoBarFade--;  
    }
}

void drawNoiseDebug() {    
    if (delayProfile <= 0) {
        for (uint8_t i = 0; i < BINS_SRC; i++) {
            if (noises[i] < fht_oct_out[i])
                noises[i] = fht_oct_out[i];
                
            String binary = String(noises[i], BIN);
            //String binary = String(i, BIN);
            for (uint8_t j = 0; j < MATRIX_COLS; j++)
                if (j < binary.length()) {
                    if (binary[binary.length() - 1 - j] == '1')
                        matrix[j][i] = strip.Color(32, 32, 32);
                    else
                        matrix[j][i] = strip.Color(0, 0, 0);
                }
                else {
                    matrix[j][i] = strip.Color(0, 0, 0);
                }
        }
    }
    else {
        delayProfile--;
    }
}

// MODE 0: EQPro
// Equalizer-like horizontal visualisation with lowest frequencies being on the left.
void drawMode_EQPro() {
    uint8_t i, j;
    
    switch (color[mode]) {

        // Basic RED
        case 0:   
        
            for (i = 0; i < 10; i++) {
                for (j = 0; j < 10; j++) {
                    if (j < bins[i]) {  
                        matrix[9-j][i] = strip.Color((brightness /  bins[i]) * (j+1), 0, 0);                
                    }
                    else {
                        matrix[9-j][i] = strip.Color(0, 0, 0);
                    }
                }
            }   
            break;  
            
        // Basic ORANGE
        case 1:   
        
            for (i = 0; i < 10; i++) {
                for (j = 0; j < 10; j++) {
                    if (j < bins[i]) {  
                        matrix[9-j][i] = strip.Color((brightness /  bins[i]) * (j+1), ((brightness / 2) /  bins[i]) * (j+1), 0);                
                    }
                    else {
                        matrix[9-j][i] = strip.Color(0, 0, 0);
                    }
                }
            }   
            break;  
            
        // Basic YELLOW
        case 2:   
        
            for (i = 0; i < 10; i++) {
                for (j = 0; j < 10; j++) {
                    if (j < bins[i]) {  
                        matrix[9-j][i] = strip.Color((brightness /  bins[i]) * (j+1), (brightness /  bins[i]) * (j+1), 0);                
                    }
                    else {
                        matrix[9-j][i] = strip.Color(0, 0, 0);
                    }
                }
            }   
            break;  
            
        // Basic LIME
        case 3:   
        
            for (i = 0; i < 10; i++) {
                for (j = 0; j < 10; j++) {
                    if (j < bins[i]) {  
                        matrix[9-j][i] = strip.Color(((brightness / 2) /  bins[i]) * (j+1), (brightness /  bins[i]) * (j+1), 0);                
                    }
                    else {
                        matrix[9-j][i] = strip.Color(0, 0, 0);
                    }
                }
            }   
            break; 
            
        // Basic GREEN
        case 4:   
        
            for (i = 0; i < 10; i++) {
                for (j = 0; j < 10; j++) {
                    if (j < bins[i]) {  
                        matrix[9-j][i] = strip.Color(0, (brightness /  bins[i]) * (j+1), 0);                
                    }
                    else {
                        matrix[9-j][i] = strip.Color(0, 0, 0);
                    }
                }
            }   
            break; 
            
        // Basic TURQUOISE
        case 5:   
        
            for (i = 0; i < 10; i++) {
                for (j = 0; j < 10; j++) {
                    if (j < bins[i]) {  
                        matrix[9-j][i] = strip.Color(0, (brightness /  bins[i]) * (j+1), ((brightness / 2) /  bins[i]) * (j+1));                
                    }
                    else {
                        matrix[9-j][i] = strip.Color(0, 0, 0);
                    }
                }
            }   
            break; 
            
        // Basic CYAN
        case 6:   
        
            for (i = 0; i < 10; i++) {
                for (j = 0; j < 10; j++) {
                    if (j < bins[i]) {  
                        matrix[9-j][i] = strip.Color(0, (brightness /  bins[i]) * (j+1), (brightness /  bins[i]) * (j+1));                
                    }
                    else {
                        matrix[9-j][i] = strip.Color(0, 0, 0);
                    }
                }
            }   
            break; 
            
        // Basic SKY BLUE
        case 7:   
        
            for (i = 0; i < 10; i++) {
                for (j = 0; j < 10; j++) {
                    if (j < bins[i]) {  
                        matrix[9-j][i] = strip.Color(0, ((brightness / 2) /  bins[i]) * (j+1), (brightness /  bins[i]) * (j+1));                
                    }
                    else {
                        matrix[9-j][i] = strip.Color(0, 0, 0);
                    }
                }
            }   
            break; 
            
        // Basic BLUE
        case 8:   
        
            for (i = 0; i < 10; i++) {
                for (j = 0; j < 10; j++) {
                    if (j < bins[i]) {  
                        matrix[9-j][i] = strip.Color(0, 0, (brightness /  bins[i]) * (j+1));                
                    }
                    else {
                        matrix[9-j][i] = strip.Color(0, 0, 0);
                    }
                }
            }   
            break; 
            
        // Basic PURPLE
        case 9:   
        
            for (i = 0; i < 10; i++) {
                for (j = 0; j < 10; j++) {
                    if (j < bins[i]) {  
                        matrix[9-j][i] = strip.Color(((brightness / 2) /  bins[i]) * (j+1), 0, (brightness /  bins[i]) * (j+1));                
                    }
                    else {
                        matrix[9-j][i] = strip.Color(0, 0, 0);
                    }
                }
            }   
            break; 
            
        // Basic MAGENTA
        case 10:   
        
            for (i = 0; i < 10; i++) {
                for (j = 0; j < 10; j++) {
                    if (j < bins[i]) {  
                        matrix[9-j][i] = strip.Color((brightness /  bins[i]) * (j+1), 0, (brightness /  bins[i]) * (j+1));                
                    }
                    else {
                        matrix[9-j][i] = strip.Color(0, 0, 0);
                    }
                }
            }   
            break; 
            
        // Basic PINK
        case 11:   
        
            for (i = 0; i < 10; i++) {
                for (j = 0; j < 10; j++) {
                    if (j < bins[i]) {  
                        matrix[9-j][i] = strip.Color((brightness /  bins[i]) * (j+1), 0, ((brightness / 2) /  bins[i]) * (j+1));                
                    }
                    else {
                        matrix[9-j][i] = strip.Color(0, 0, 0);
                    }
                }
            }   
            break; 
            
        // Basic WHITE
        case 12:   
        
            for (i = 0; i < 10; i++) {
                for (j = 0; j < 10; j++) {
                    if (j < bins[i]) {  
                        matrix[9-j][i] = strip.Color((brightness /  bins[i]) * (j+1), (brightness /  bins[i]) * (j+1), (brightness /  bins[i]) * (j+1));                
                    }
                    else {
                        matrix[9-j][i] = strip.Color(0, 0, 0);
                    }
                }
            }   
            break; 

        // GREEN fading to RED by individual volumes
        case 13:
        
            for (i = 0; i < 10; i++) {
                for (j = 0; j < 10; j++) {
                    if (j < bins[i]) {                      
                        matrix[9-j][i] = strip.Color(brightness / 10 * bins[i], max(brightness - brightness / 10 * bins[i], 0), 0);
                    }
                    else {
                        matrix[9-j][i] = strip.Color(0, 0, 0);
                    }
                }
            }
            break;

        // BLUE with PURPLE peaks
        case 14:   
        
            for (i = 0; i < 10; i++) {
                for (j = 0; j < 10; j++) {
                    if (j < bins[i]) {                  
                        if (j == bins[i] - 1)
                            matrix[9-j][i] = strip.Color(brightness / 2, 0, brightness);
                        else
                            matrix[9-j][i] = strip.Color(0, 0, ((brightness / 2) / bins[i]) * (j+1));
                    }
                    else {
                        matrix[9-j][i] = strip.Color(0, 0, 0);
                    }
                }
            }   
            break;  
             
    }
}


// Matrix pushing
void pushMatrix() {  
    uint16_t i;      
    for (i = 0; i < MATRIX_ROWS * MATRIX_COLS; i++) {
        if ((i / MATRIX_ROWS) % 2 == 0) {
            //Serial.println("Drawing: [" + String(i / MATRIX_ROWS * 10 + 9 - i) + "][" + String(i / MATRIX_ROWS) + "]");
            strip.setPixelColor(i, matrix[i / MATRIX_ROWS * 10 + 9 - i][i / MATRIX_ROWS]);
        }
        else {
            //Serial.println("Drawing: [" + String(i % MATRIX_ROWS) + "][" + String(i / MATRIX_ROWS) + "]");
            strip.setPixelColor(i, matrix[i % MATRIX_ROWS][i / MATRIX_ROWS]);
        }
    }
    strip.show(); 
}

void setup() {  
    Serial.begin(115200);

    setupSettings();
    setupPins();
    setupMicrophone();
    setupRegisters();      
    
    strip.begin();
    strip.show();
}

void loop() {      
    while(1) {
        processFHT();
        //Serial.println(fht_oct_out[5]); 
        
        if (!PROFILE_NOISES) {
            processBins(); 
            drawVisualiser();
        }
        else {
            drawNoiseDebug();
        }
        
        pushMatrix(); 
    
        processButtons();
    } 
}
