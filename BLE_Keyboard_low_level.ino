#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "BLE2902.h"
#include "BLEHIDDevice.h"
#include "HIDTypes.h"
#include "HIDKeyboardTypes.h"
#include <driver/adc.h>
#include "sdkconfig.h"


#include "keyboardData.h" 

//
//  inspired by   : https://github.com/nkolban/esp32-snippets/issues/230#issuecomment-473135679
//

BLEHIDDevice* hid;

const char keyboardName[]="RemoteSpace";

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

// http://www.keil.com/forum/15671/usb-mouse-with-scroll-wheel/
// Wheel Mouse - simplified version - 5 button, vertical and horizontal wheel
//
// Input report - 5 bytes
//
//     Byte | D7      D6      D5      D4      D3      D2      D1      D0
//    ------+---------------------------------------------------------------------
//      0   |  0       0       0    Forward  Back    Middle  Right   Left (Buttons)
//      1   |                             X
//      2   |                             Y
//      3   |                       Vertical Wheel
//      4   |                    Horizontal (Tilt) Wheel
//
// Feature report - 1 byte
//
//     Byte | D7      D6      D5      D4   |  D3      D2  |   D1      D0
//    ------+------------------------------+--------------+----------------
//      0   |  0       0       0       0   |  Horizontal  |    Vertical
//                                             (Resolution multiplier)
//
// Reference
//    Wheel.docx in "Enhanced Wheel Support in Windows Vista" on MS WHDC
//    http://www.microsoft.com/whdc/device/input/wheel.mspx
//


const int button1Pin = 39;     // the number of the pushbutton pin
const int button2Pin = 38;     // the number of the pushbutton pin
const int ledPin =     5;      // the number of the LED pin

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");

  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);

  xTaskCreate(taskServer, "server", 20000, NULL, 5, NULL);
}

void loop() {
  if(connected){

    //vTaskDelay(5000);Serial.println("dormindo");
    if (true) {
      Serial.println("key space");
      keyboard_write(' ');
      delay(10);

      delay(5000);
      Serial.println("key space");
      keyboard_write(' ');
      delay(5000);
    } else {
      while (digitalRead(button2Pin) ==  LOW ){ 
        Serial.println("key space");
        keyboard_write(' ');
        delay(10);
      }
      while (digitalRead(button1Pin) ==  LOW ){ 
        Serial.println("key space");
        keyboard_write(' ');
        delay(10);
      }
    }
  }
  delay(50);
}

//  Low level key report: up to 6 keys and shift, ctrl etc at once
typedef struct {
    uint8_t modifiers;
    uint8_t reserved;
    uint8_t keys[6];
} KeyReport;

typedef uint8_t MediaKeyReport[2];

void keyboard_sendReport(KeyReport* keys);

void keyboard_sendReport(MediaKeyReport* keys);

const MediaKeyReport KEY_MEDIA_NEXT_TRACK = {1, 0};
const MediaKeyReport KEY_MEDIA_PREVIOUS_TRACK = {2, 0};
const MediaKeyReport KEY_MEDIA_STOP = {4, 0};
const MediaKeyReport KEY_MEDIA_PLAY_PAUSE = {8, 0};
const MediaKeyReport KEY_MEDIA_MUTE = {16, 0};
const MediaKeyReport KEY_MEDIA_VOLUME_UP = {32, 0};
const MediaKeyReport KEY_MEDIA_VOLUME_DOWN = {64, 0};
const MediaKeyReport KEY_MEDIA_WWW_HOME = {128, 0};
const MediaKeyReport KEY_MEDIA_LOCAL_MACHINE_BROWSER = {0, 1}; // Opens "My Computer" on Windows
const MediaKeyReport KEY_MEDIA_CALCULATOR = {0, 2};
const MediaKeyReport KEY_MEDIA_WWW_BOOKMARKS = {0, 4};
const MediaKeyReport KEY_MEDIA_WWW_SEARCH = {0, 8};
const MediaKeyReport KEY_MEDIA_WWW_STOP = {0, 16};
const MediaKeyReport KEY_MEDIA_WWW_BACK = {0, 32};
const MediaKeyReport KEY_MEDIA_CONSUMER_CONTROL_CONFIGURATION = {0, 64}; // Media Selection
const MediaKeyReport KEY_MEDIA_EMAIL_READER = {0, 128};


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
