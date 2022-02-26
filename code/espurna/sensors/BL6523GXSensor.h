// -----------------------------------------------------------------------------
// BL6523GX based power monitor/watt meter
// Copyright (C) 2022 by Jeevas Vasudevan and the Internet
// -----------------------------------------------------------------------------
// Connection Diagram:
// -------------------
//
// We are only passively sniffing both RX and TX of BL6523GX to get the data
// *BL6523GX will be initialized and polled by the unit MCU*
//
// +---------+
// | ESPurna |                                          
// |   Node  |                                             
// | G  4  5 |                                               
// +-+--+--+-+                                               
//   |  |  |                                                 
//   |  |  +------- R -------+
//   |  +------- R -------+  |
//   +-----------------+  |  |
//                     |  |  | 
//                     |  |  |  
//                     |  |  | 
//                   +------------+ 
//                   | G  Tx Rx   | 
//                   | BL6523GX   | 
//                   | WattMeter  |   
//                   +------------+  
//
// Where:
// ------
//     G = GND
//     4 = ESPurna GPIO4 ->1K-> BL6523GX Tx
//     5 = ESPurna GPIO5 ->1K-> BL6523GX Rx
//     R = Resistor  1K
//
// More Info:
// ----------
//     See ESPurna Wiki - https://github.com/xoseperez/espurna/wiki/Sensor-BL6523GX
//
// Reference:
// ----------
//     https://datasheet4u.com/datasheet-pdf/BELLING/BL6523GX/pdf.php?id=930160


#if SENSOR_SUPPORT && BL6523GX_SUPPORT

#pragma once

#include "BaseSensor.h"
#include "BaseEmonSensor.h"

#include <SoftwareSerial.h>

#define BL6523_RX_DATASET_SIZE 2
#define BL6523_TX_DATASET_SIZE 4
#define BL6523_REG_AMPS 0x05
#define BL6523_REG_VOLTS 0x07
#define BL6523_REG_FREQ  0x09
#define BL6523_REG_WATTS 0x0A
#define BL6523_REG_POWF  0x08
#define BL6523_REG_WATTHR 0x0C

#define SINGLE_PHASE 0
#define RX_WAIT 100

#define BL6523_IREF 297899
#define BL6523_UREF 13304
#define BL6523_FREF 3907
#define BL6523_PREF 707
#define BL6523_PWHRREF_D 33 // Substract this from BL6523_PREF to get WattHr Div.

class BL6523GXSensor : public BaseEmonSensor {

    public:

        // ---------------------------------------------------------------------
        // Public
        // ---------------------------------------------------------------------

        static constexpr Magnitude Magnitudes[] {
            MAGNITUDE_CURRENT,
            MAGNITUDE_VOLTAGE,
            MAGNITUDE_POWER_ACTIVE,
            MAGNITUDE_POWER_FACTOR,
            MAGNITUDE_ENERGY
        };

        BL6523GXSensor() {
            _sensor_id = SENSOR_BL6523GX_ID;
            _count = std::size(Magnitudes);
            findAndAddEnergy(Magnitudes);
        }

        // ---------------------------------------------------------------------

        void setRX(unsigned char pin_rx) {
            if (_pin_rx == pin_rx) return;
            _pin_rx = pin_rx;
            _dirty = true;
        }

        void setInvertedRX(bool inverted_rx) {
            if (_inverted_rx == inverted_rx) return;
            _inverted_rx = inverted_rx;
            _dirty = true;
        }
        void setTX(unsigned char pin_tx) {
            if (_pin_tx == pin_tx) return;
            _pin_tx = pin_tx;
            _dirty = true;
        }

        void setInvertedTX(bool inverted_tx) {
            if (_inverted_tx == inverted_tx) return;
            _inverted_tx = inverted_tx;
            _dirty = true;
        }

        // ---------------------------------------------------------------------

        unsigned char getRX() {
            return _pin_rx;
        }

        bool getInvertedRX() {
            return _inverted_rx;
        }

        unsigned char getTX() {
            return _pin_tx;
        }

        bool getInvertedTX() {
            return _inverted_tx;
        }

        // ---------------------------------------------------------------------

        double getRatio(unsigned char index) const override {
            switch (index) {
            case 0:
                return _current_ratio;
            case 1:
                return _voltage_ratio;
            case 2:
                return _power_active_ratio;
            }

            return BaseEmonSensor::getRatio(index);
        }

        void setRatio(unsigned char index, double value) override {
            if (value > 0.0) {
                switch (index) {
                case 0:
                    _current_ratio = value;
                    break;
                case 1:
                    _voltage_ratio = value;
                    break;
                case 2:
                    _power_active_ratio = value;
                    break;
                }
            }
        }

        // ---------------------------------------------------------------------
        // Sensor API
        // ---------------------------------------------------------------------

        // Initialization method, must be idempotent
        void begin() {

            resetRatios();

            if (!_dirty) return;

            if (_serial_rx) {
                _serial_rx.reset(nullptr);
            }

            if (3 == _pin_rx) {
                Serial.begin(BL6523GX_BAUDRATE);
            } else if (13 == _pin_rx) {
                Serial.begin(BL6523GX_BAUDRATE);
                Serial.flush();
                Serial.swap();
            } else {
                _serial_rx = std::make_unique<SoftwareSerial>(_pin_rx, -1, _inverted_rx);
                _serial_rx->enableIntTx(false);
                _serial_rx->begin(BL6523GX_BAUDRATE);
            }

            if (3 == _pin_tx) {
                Serial.begin(BL6523GX_BAUDRATE);
            } else if (13 == _pin_tx) {
                Serial.begin(BL6523GX_BAUDRATE);
                Serial.flush();
                Serial.swap();
            } else {
                _serial_tx = std::make_unique<SoftwareSerial>(_pin_tx, -1, _inverted_tx);
                _serial_tx->enableIntTx(false);
                _serial_tx->begin(BL6523GX_BAUDRATE);
            }

            _ready = true;
            _dirty = false;

        }

        // Descriptive name of the sensor
        String description() {
            char buffer[28];
            if (_serial_tx_is_hardware()) {
                snprintf(buffer, sizeof(buffer), "BL6523GX TX @ HwSerial");
            } else {
                snprintf(buffer, sizeof(buffer), "BL6523GX TX @ SwSerial(%u,%u,NULL)", _pin_rx, _pin_tx);
            }
            return String(buffer);
        }

        // Descriptive name of the slot # index
        String description(unsigned char index) {
            return description();
        };

        // Address of the sensor (it could be the GPIO or I2C address)
        String address(unsigned char index) {
            return String(_pin_rx);
        }

        // Loop-like method, call it in your main loop
        void tick() {
            _read();
        }

        // Type for slot # index
        unsigned char type(unsigned char index) {
            if (index < std::size(Magnitudes)) {
                return Magnitudes[index].type;
            }

            return MAGNITUDE_NONE;
        }

        // Current value for slot # index
        double value(unsigned char index) {  
            if (index == 0) return _current;
            if (index == 1) return _voltage;
            if (index == 2) return _active;
            if (index == 3) return _reactive;
            if (index == 4) return _voltage * _current;
            if (index == 5) return ((_voltage > 0) && (_current > 0)) ? 100 * _active / _voltage / _current : 100;
            if (index == 6) return _energy;
            return 0;
        }

    protected:

        // ---------------------------------------------------------------------
        // Protected
        // ---------------------------------------------------------------------


        bool _read() {

            _error = SENSOR_ERROR_OK;

            //static unsigned char index = 0;
            //static unsigned long last = millis();
            uint32_t powf_word = 0, powf_buf = 0, i = 0;
            float powf = 0.0f;

            if (!_serial_rx_available())
            {
                #if SENSOR_DEBUG
                DEBUG_MSG("BL6:No Rx Data available" );
                #endif
                return false;
            }

            while ((_serial_rx_peek() != 0x35) && _serial_rx_available())
            {
                _serial_rx_read();
            }

            if (_serial_rx_available() < BL6523_RX_DATASET_SIZE)
            {
                #if SENSOR_DEBUG
                DEBUG_MSG("BL6:Rx less than expected" );
                #endif
                return false;
            }

            uint8_t rx_buffer[BL6523_RX_DATASET_SIZE];
            _serial_rx_readbytes(rx_buffer, BL6523_RX_DATASET_SIZE);
            _serial_rx_flush(); // Make room for another burst

            //DEBUG_MSG(LOG_LEVEL_DEBUG_MORE, rx_buffer, BL6523_RX_DATASET_SIZE);
            
            i=0;
            while (_serial_tx_available() < BL6523_TX_DATASET_SIZE)
            {
                // sleep till TX buffer is full
                delay(10);
                if ( i++ > RX_WAIT ){
                break;
                }
            }
            

            uint8_t tx_buffer[BL6523_TX_DATASET_SIZE];
            _serial_tx_readbytes(tx_buffer, BL6523_TX_DATASET_SIZE);
            _serial_tx_flush(); // Make room for another burst
            
            #if SENSOR_DEBUG
            DEBUG_MSG(tx_buffer, BL6523_TX_DATASET_SIZE);
            #endif
            
            /* Checksum: （Addr+Data_L+Data_M+Data_H） & 0xFF, then byte invert */
            uint8_t crc = rx_buffer[1]; //Addr
            for (uint32_t i = 0; i < (BL6523_TX_DATASET_SIZE - 1); i++)
            {
                crc += tx_buffer[i]; //Add Data_L,Data_M and Data_H to Addr
            }
            crc &= 0xff; // Bitwise AND 0xFF
            crc = ~crc; // Invert the byte
            if (crc != tx_buffer[BL6523_TX_DATASET_SIZE - 1])
            {
                #if SENSOR_DEBUG
                DEBUG_MSG("BL6:Checksum failure");
                #endif
                _serial_tx_flush(); 
                _serial_rx_flush(); 
                return false;
            }

            /* WRITE DATA (format: command(write->0xCA) address data_low data_mid data_high checksum )
            WRITE Sample(RX):
            RX: CA 3E 55 00 00 6C (WRPROT - allow)
            RX: CA 14 00 00 10 DB (MODE) 
            RX: CA 15 04 00 00 E6 (GAIN - IB 16x gain )
            RX: CA 19 08 00 00 DE (WA_CFDIV )
            RX: CA 3E AA 00 00 17 (WRPROT - disable)
            */

            /* READ DATA (format: command(read->0x35) address data_low data_mid data_high checksum )
            READ Sample(RX-TX) Data:
            RX: 35 05 TX: E4 00 00 16 (IA rms )
            RX: 35 07 TX: D5 A3 2E 52 (V rms )
            RX: 35 09 TX: F0 FB 02 09 (FREQ) 
            RX: 35 0A TX: 00 00 00 F5 (WATT) 
            RX: 35 08 TX: 00 00 00 F7 (PF)
            RX: 35 0C TX: 00 00 00 F3 (WATT_HR)
            */

            switch(rx_buffer[1]) {
            case BL6523_REG_AMPS :
                _current =  (float)((tx_buffer[2] << 16) | (tx_buffer[1] << 8) | tx_buffer[0]) / BL6523_IREF;     // 1.260 A
                break;
            case BL6523_REG_VOLTS :
                _voltage = (float)((tx_buffer[2] << 16) | (tx_buffer[1] << 8) | tx_buffer[0]) / BL6523_UREF;     // 230.2 V
                break;
            case BL6523_REG_FREQ :
                _frequency = (float)((tx_buffer[2] << 16) | (tx_buffer[1] << 8) | tx_buffer[0]) / BL6523_FREF;    // 50.0 Hz
                break;        
            case BL6523_REG_WATTS :
                _power_active = (float)((tx_buffer[2] << 16) | (tx_buffer[1] << 8) | tx_buffer[0]) / BL6523_PREF; // -196.3 W
                break;
            case BL6523_REG_POWF :
            /* Power factor =(sign bit)*((PF[22]×2^－1）＋（PF[21]×2^－2）＋。。。)
                Eg., reg value  0x7FFFFF(HEX) -> PF 1, 0x800000(HEX) -> -1, 0x400000(HEX) -> 0.5
                */
            powf = 0.0f;
            powf_buf = ((tx_buffer[2]  << 16) | (tx_buffer[1] << 8) | tx_buffer[0]);
            powf_word = (powf_buf >> 23) ? ~(powf_buf & 0x7fffff) : powf_buf & 0x7fffff; //Extract the 23 bits and invert if sign bit(24) is set
            for (int i = 0; i < 23; i++){ // Accumulate powf from 23 bits
                powf += ((powf_word >> (22-i)) * pow(2,(0-(i+1)))); 
                powf_word = powf_word & (0x7fffff >> (1+i));
            }
            powf = (powf_buf >> 23) ? (0.0f - (powf)) : powf; // Negate if sign bit(24) is set
            _power_factor = powf;
                break;
            case BL6523_REG_WATTHR :
                _energy = (float)((tx_buffer[2] << 16) | (tx_buffer[1] << 8) | tx_buffer[0]) / ( BL6523_PREF - BL6523_PWHRREF_D ); // 6.216 kWh => used in EnergyUpdateTotal()
                break;
            default :  
            break;            
            }
            //Energy.data_valid[SINGLE_PHASE] = 0;
            //EnergyUpdateTotal();
        }

        // ---------------------------------------------------------------------

        bool _serial_tx_is_hardware() const {
            return (3 == _pin_tx) || (13 == _pin_tx);
        }

        bool _serial_tx_available() {
            if (_serial_tx_is_hardware()) {
                return Serial.available();
            } else {
                return _serial_tx->available();
            }
        }

        void _serial_tx_flush() {
            if (_serial_tx_is_hardware()) {
                return Serial.flush();
            } else {
                return _serial_tx->flush();
            }
        }

        uint8_t _serial_tx_read() {
            if (_serial_tx_is_hardware()) {
                return Serial.read();
            } else {
                return _serial_tx->read();
            }
        }

        uint8_t _serial_tx_peek() {
            if (_serial_tx_is_hardware()) {
                return Serial.peek();
            } else {
                return _serial_tx->peek();
            }
        }

         size_t _serial_tx_readbytes(uint8_t *buffer, size_t num_bytes) {
            if (_serial_tx_is_hardware()) {
                return Serial.readBytes(buffer, num_bytes);
            } else {
                return _serial_tx->readBytes(buffer, num_bytes);
            }
        }

        bool _serial_rx_is_hardware() const {
            return (3 == _pin_rx) || (13 == _pin_rx);
        }

        bool _serial_rx_available() {
            if (_serial_rx_is_hardware()) {
                return Serial.available();
            } else {
                return _serial_rx->available();
            }
        }

        void _serial_rx_flush() {
            if (_serial_rx_is_hardware()) {
                return Serial.flush();
            } else {
                return _serial_rx->flush();
            }
        }

        uint8_t _serial_rx_read() {
            if (_serial_rx_is_hardware()) {
                return Serial.read();
            } else {
                return _serial_rx->read();
            }
        }

        uint8_t _serial_rx_peek() {
            if (_serial_rx_is_hardware()) {
                return Serial.peek();
            } else {
                return _serial_rx->peek();
            }
        }
         size_t _serial_rx_readbytes( uint8_t *buffer, size_t num_bytes) {
            if (_serial_rx_is_hardware()) {
                return Serial.readBytes(buffer, num_bytes);
            } else {
                return _serial_rx->readBytes(buffer, num_bytes);
            }
        }       

        // ---------------------------------------------------------------------

        int _pin_rx = BL6523GX_RX_PIN;
        int _pin_tx = BL6523GX_TX_PIN;
        bool _inverted_rx = BL6523GX_RX_PIN_INVERSE;
        bool _inverted_tx = BL6523GX_TX_PIN_INVERSE;
        std::unique_ptr<SoftwareSerial> _serial_tx;
        std::unique_ptr<SoftwareSerial> _serial_rx;

        double _active = 0;
        double _reactive = 0;
        double _voltage = 0;
        double _current = 0;
        double _power_active { 0.0 };
        double _energy { 0.0 };
        double _frequency { 0.0 };
        double _power_factor { 0.0 };

        unsigned char _data[24] {0};

};

#if __cplusplus < 201703L
constexpr BaseEmonSensor::Magnitude BL6523GXSensor::Magnitudes[];
#endif

#endif // SENSOR_SUPPORT && BL6523GX_SUPPORT
