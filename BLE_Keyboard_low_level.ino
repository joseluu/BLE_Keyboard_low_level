#define CONFIG_ARDUHAL_LOG_DEFAULT_LEVEL 3
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "BLE2902.h"
#include "BLEHIDDevice.h"
#include "HIDTypes.h"
#include "HIDKeyboardTypes.h"
#include <driver/adc.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp32-hal-log.h"

//#include "esp_app_trace.h"

// configuration for esp32s3 supermini
// Device: ESP32S3 Dev module
// USB CDC On boot enabled
// Jtag adapter disabled
// Upload mode UART0/ hardware CDC
// Usb mode: Hardware CDC and jtag

#include <Bounce2.h>

#include "keyboardData.h" 


//
//  inspired by   : https://github.com/nkolban/esp32-snippets/issues/230#issuecomment-473135679
//

BLEHIDDevice* hid;

const char keyboardName[]="Dance_Remote_Jose";
static const char* TAG = &keyboardName[0];

bool connected = false;
static BLECharacteristic *inputKeyboard = nullptr;
static BLECharacteristic *inputMediaKeys = nullptr;
static BLECharacteristic *outputKeyboard = nullptr;

bool isConnected(void){
  return connected;
}
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer){
    connected = true;
    BLE2902* desc = (BLE2902*)inputKeyboard->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
    desc->setNotifications(true);
    desc = (BLE2902*)inputMediaKeys->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
    desc->setNotifications(true);
  }

  void onDisconnect(BLEServer* pServer){
    connected = false;
    BLE2902* desc = (BLE2902*)inputKeyboard->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
    desc->setNotifications(false);
    desc = (BLE2902*)inputMediaKeys->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
    desc->setNotifications(false);
  }
};

#define BLE_SERVER_CONN_PARAMS_TYPE     esp_ble_gatts_cb_param_t*
class CharacteristicCallbacks : public BLECharacteristicCallbacks
{
    void onRead(BLECharacteristic* pCharacteristic, BLE_SERVER_CONN_PARAMS_TYPE connInfo) override
    {
    }

    void onWrite(BLECharacteristic* pCharacteristic, BLE_SERVER_CONN_PARAMS_TYPE connInfo) override
    {
    }
};

BLEServerCallbacks *pServerCallbacks = nullptr;
BLECharacteristicCallbacks  *pCharacteristicCallbacks = nullptr;

void taskServer(void*){


    BLEDevice::init(keyboardName);
    BLEServer *pServer = BLEDevice::createServer();
    pServerCallbacks = new ServerCallbacks();
    if (!pServerCallbacks) {
        log_e("Memory request failed, unable to create a new ServerCallbacks object");
        return;
    }

    hid = new BLEHIDDevice(pServer);
    if (!hid) {
        log_e("Memory request failed, unable to create a new HID object");
        delete pServerCallbacks;
        pServerCallbacks = nullptr;
        return;
    }

    pCharacteristicCallbacks =  new CharacteristicCallbacks();
    if (!pCharacteristicCallbacks) {
        log_e("Memory request failed, unable to create a new CharacteristicCallbacks object");
        delete hid;
        hid = nullptr;
        delete pServerCallbacks;
        pServerCallbacks = nullptr;
        return;
    }

    pServer->setCallbacks(pServerCallbacks);
    inputKeyboard = hid->inputReport(KEYBOARD_ID);  // <-- input REPORTID from report map
    outputKeyboard = hid->outputReport(KEYBOARD_ID);
    inputMediaKeys = hid->inputReport(MEDIA_KEYS_ID);
    outputKeyboard->setCallbacks(pCharacteristicCallbacks);

    std::string name = "Jose";
    hid->manufacturer()->setValue(name.c_str());
    uint16_t vid       = 0x05ac;
    uint16_t pid       = 0x820a;
    uint16_t version   = 0x0210;
    hid->pnp(0x02, vid, pid, version);
    hid->hidInfo(0x00,0x01);

  BLESecurity *pSecurity = new BLESecurity();

  pSecurity->setAuthenticationMode(ESP_LE_AUTH_BOND);
  hid->reportMap((uint8_t*)reportMapKeyboard, sizeof(reportMapKeyboard));
  hid->startServices();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->setAppearance(HID_KEYBOARD);
  pAdvertising->addServiceUUID(hid->hidService()->getUUID());
  pAdvertising->start();
  hid->setBatteryLevel(17);

  ESP_LOGD(LOG_TAG, "Advertising started!");
  delay(portMAX_DELAY);

};

const int button1Pin = 4;     // the number of the pushbutton pin
const int button2Pin = 5;     // the number of the pushbutton pin
const int button3Pin = 6;     // the number of the pushbutton pin
const int button4Pin = 7;     // the number of the pushbutton pin
const int ledPin =     48;      // the number of the LED pin

//Button button1(button1Pin); // Connect your button between pin 2 and GND
Bounce2::Button button1 = Bounce2::Button();
Bounce2::Button button2 = Bounce2::Button();
Bounce2::Button button3 = Bounce2::Button();
Bounce2::Button button4 = Bounce2::Button();

void setup() {                                             
  Serial.begin(115200);
  Serial.println();
  Serial.println("Starting BLE work!");

    //button1.begin();
    button1.attach(button1Pin, INPUT_PULLUP);
    button1.interval(50);
    button1.setPressedState(LOW);
    button2.attach(button2Pin, INPUT_PULLUP);
    button2.interval(50);
    button2.setPressedState(LOW); 
    button3.attach(button3Pin, INPUT_PULLUP);
    button3.interval(50);
    button3.setPressedState(LOW); 
    button4.attach(button4Pin, INPUT_PULLUP);
    button4.interval(50);
    button4.setPressedState(LOW); 

  pinMode(ledPin, OUTPUT);

  xTaskCreate(taskServer, "server", 20000, NULL, 5, NULL);
}

static bool button1_pressed;

void loop() {
    button1.update();
    button2.update();
    button3.update();
    button4.update();
            //vTaskDelay(5000);Serial.println("dormindo");
    if(connected){
        if (button1.rose()) {
            digitalWrite(ledPin, HIGH);
            Serial.println("key space");
            keyboard_write(' ');
            delay(10);
        }
        if (button2.rose()) {
            digitalWrite(ledPin, HIGH);
            Serial.println("key up arrow");
            keyboard_write(0xDA);
            delay(10);
        }
        if (button3.rose()) {
            digitalWrite(ledPin, HIGH);
            Serial.println("key down arrow");
            keyboard_write(0xD9);
            delay(10);
        }
        if (button4.rose()) {
            digitalWrite(ledPin, HIGH);
            Serial.println("key home");
            keyboard_write(0xD2);
            delay(10);
        }
    }
    delay(50);
}

// keys are defined here:
// AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.2.0\libraries\BLE\src/HIDKeyboardTypes.h:54


//  Low level key report: up to 6 keys and shift, ctrl etc at once
typedef struct {
    uint8_t modifiers;
    uint8_t reserved;
    uint8_t keys[6];
} KeyReport;

typedef uint8_t MediaKeyReport[2];

void keyboard_sendReport(KeyReport* keys);

void keyboard_sendReport(MediaKeyReport* keys);


KeyReport          _keyReport;
MediaKeyReport     _mediaKeyReport;



void setWriteError(void){
  Serial.println("a write error occurred");
}
// press() adds the specified key (printing, non-printing, or modifier)
// to the persistent key report and sends the report.  Because of the way
// USB HID works, the host acts like the key remains pressed until we
// call release(), releaseAll(), or otherwise clear the report and resend.
size_t press(uint8_t k)
{
    uint8_t i;
    if (k >= 136) {         // it's a non-printing key (not a modifier)
        k = k - 136;
    } else if (k >= 128) {  // it's a modifier key
        _keyReport.modifiers |= (1 << (k - 128));
        k = 0;
    } else {                // it's a printing key
        k = pgm_read_byte(_asciimap + k);
        if (!k) {
            setWriteError();
            return 0;
        }
        if (k & 0x80) {                     // it's a capital letter or other character reached with shift
            _keyReport.modifiers |= 0x02;   // the left shift modifier
            k &= 0x7F;
        }
    }

    // Add k to the key report only if it's not already present
    // and if there is an empty slot.
    if (_keyReport.keys[0] != k && _keyReport.keys[1] != k &&
            _keyReport.keys[2] != k && _keyReport.keys[3] != k &&
            _keyReport.keys[4] != k && _keyReport.keys[5] != k) {

        for (i = 0; i < 6; i++) {
            if (_keyReport.keys[i] == 0x00) {
                _keyReport.keys[i] = k;
                break;
            }
        }
        if (i == 6) {
            setWriteError();
            return 0;
        }
    }
    keyboard_sendReport(&_keyReport);
    return 1;
}

size_t release(uint8_t k)
{
    uint8_t i;
    if (k >= 136) {         // it's a non-printing key (not a modifier)
        k = k - 136;
    } else if (k >= 128) {  // it's a modifier key
        _keyReport.modifiers &= ~(1 << (k - 128));
        k = 0;
    } else {                // it's a printing key
        k = pgm_read_byte(_asciimap + k);
        if (!k) {
            return 0;
        }
        if (k & 0x80) {                         // it's a capital letter or other character reached with shift
            _keyReport.modifiers &= ~(0x02);    // the left shift modifier
            k &= 0x7F;
        }
    }

    // Test the key report to see if k is present.  Clear it if it exists.
    // Check all positions in case the key is present more than once (which it shouldn't be)
    for (i = 0; i < 6; i++) {
        if (0 != k && _keyReport.keys[i] == k) {
            _keyReport.keys[i] = 0x00;
        }
    }

    keyboard_sendReport(&_keyReport);
    return 1;
}

size_t keyboard_write(uint8_t c)
{
    uint8_t p = press(c);  // Keydown
    release(c);            // Keyup
    return p;              // just return the result of press() since release() almost always returns 1
}


void keyboard_sendReport(KeyReport* keys)
{
    if (isConnected() && inputKeyboard) {
        inputKeyboard->setValue((uint8_t*)keys, sizeof(KeyReport));
        inputKeyboard->notify();
    }
}

void keyboard_sendReport(uint8_t* keys)
{
    if (isConnected() && inputMediaKeys) {
        inputMediaKeys->setValue((uint8_t*)keys, sizeof(MediaKeyReport));
        inputMediaKeys->notify();
    }
}
