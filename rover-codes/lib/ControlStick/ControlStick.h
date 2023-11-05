#ifndef _CONTROLSTICK_H_
#define _CONTROLSTICK_H_

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "HIDParser.h"

/*
*****************************************************************************************
* CONSTANTS
*****************************************************************************************
*/

/*
*****************************************************************************************
* MACROS & STRUCTURES
*****************************************************************************************
*/

/*
*****************************************************************************************
* CLASS
*****************************************************************************************
*/
class ButtonTracker {
public:
    ButtonTracker(int shift=0) {
        _shift   = shift;
        _btn     = 0;
        _oldBtn  = 0;
    }

    void begin(int btn) {
        _btn = btn;
        _toggled   = btn ^ _oldBtn;
        _cur_shift = btn & _shift;
    }

    void end() {
        _oldBtn = _btn;
    }

    void setShift(int shift) {
        _shift = shift;
    }

    bool isPressed(int check) {
        int shift  = check & _shift;

        if (shift != _cur_shift)
            return false;

        int toggled = _toggled & (~_shift);     // clear shift mask
        return bool((toggled & check) && (_btn & check));
    }

    bool isReleased(int check) {
        int shift  = check & _shift;

        if (shift != _cur_shift)
            return false;

        int toggled = _toggled & (~_shift);     // clear shift mask
        return bool((toggled & check) && !(_btn & check));
    }

    bool isToggled(int check) {
        int shift  = check & _shift;

        if (shift != _cur_shift)
            return false;

        int toggled = _toggled & (~_shift);     // clear shift mask
        return bool(toggled & check);
    }

    bool isToggled() {
        return bool(_toggled);
    }

    bool isOn(int check) {
        return bool(_btn & check);
    }

    bool isOff(int check) {
        return bool(!(_btn & check));
    }

private:
    int _shift;
    int _cur_shift;
    int _btn;
    int _toggled;
    int _oldBtn;
};

class StickCallback {
public:
    StickCallback()             {}
    virtual void onConnect()    {}
    virtual void onDisconnect() {}
    virtual void onStickChanged(int axisX, int axisY, int axisZ, int axisRZ, int axisLT, int axisRT, int dpad, int btns)    {}
};


class AdvertisedDeviceCallbacks;
class BLEJoyClientCallback;

class ControlStick {
public:
    enum {
        BTN_A = 0,
        BTN_B,
        BTN_MENU,
        BTN_X,
        BTN_Y,
        BTN_RSV1,
        BTN_L1,
        BTN_R1,

        BTN_L2 = 8,
        BTN_R2,
        BTN_SEL,
        BTN_START,
        BTN_POWER,
        BTN_LTHUMB,
        BTN_RTHUMB,
        BTN_RSV3,

        BTN_DPAD_UP = 16,
        BTN_DPAD_RIGHT,
        BTN_DPAD_DOWN,
        BTN_DPAD_LEFT
    } BTN_T;

    ControlStick();
    virtual ~ControlStick() { }

    void add(BLEAddress* addr, BLEUUID uuidService, BLEUUID uuidCharacteristic, notify_callback cb);
    void begin();
    bool connect();
    bool isConnecting() { return _isConnecting; }
    bool isConnected()  { return _isConnected; }
    void rescan();
    void stop();

    void addSupportedDevices();
    void setStickCallback(StickCallback *callback)   { _pStickCallback = callback; }

    friend class AdvertisedDeviceCallbacks;
    friend class BLEJoyClientCallback;

protected:
    std::vector <BLEAddress*>       _reqDevAddrs;
    std::vector <BLEUUID>           _uuidServices;
    std::vector <BLEUUID>           _uuidCharacteristics;
    std::vector <notify_callback>   _notifyCallbacks;

    BLEAddress*                 _pServerAddress;
    boolean                     _isConnecting;
    boolean                     _isConnected;
    BLERemoteCharacteristic*    _pRemoteCharacteristic;
    HID_ReportInfo_t            _hidReport;
    HID_ReportItem_t            *_hidItemInfo[6];

    StickCallback*              _pStickCallback;

    void cbHidBLE(NimBLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);
    void cbFlyPadBLE(NimBLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);
    void cbGameSirT1DBLE(NimBLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);


private:
    int getBit(int val, int bit) { return ((val >> bit) & 1); }
    int setBit(int val, int bit) { return (val << bit);       }

    void getHIDItemInfo(int page, int usage, HID_ReportItem_t **pInfo);
    void parseHIDData(uint8_t *pData);
    int  getHIDValue(int idx);

    BLEUUID                     _uuidFoundService;
    int                         _nFoundIdx;
    int                         _nConnTryCtr;
    NimBLEClient*               _pClient;
};

#if 0
class JoystickBLE : public ControlStick {
public:
    // this is the service UUID of the VR Control handheld mouse/joystick device (HID)
    //"00001812-0000-1000-8000-00805f9b34fb"

    // this characteristic UUID works for joystick & triggers (report)
    //"00002A4D-0000-1000-8000-00805f9b34fb"
    JoystickBLE(BLEUUID uuidService = BLEUUID("1812"), BLEUUID uuidCharacteristic = BLEUUID("2A4D")) : ControlStick(uuidService, uuidCharacteristic) {
    }

    ~JoystickBLE() {
    }
};

class FlyPadBLE : public ControlStick {
public:
    FlyPadBLE(BLEUUID uuidService = BLEUUID("9e35fa00-4344-44d4-a2e2-0c7f6046878b"), BLEUUID uuidCharacteristic = BLEUUID("9e35fa01-4344-44d4-a2e2-0c7f6046878b")) : ControlStick(uuidService, uuidCharacteristic) {
    }

    ~FlyPadBLE() {
    }
};
#endif
#endif
