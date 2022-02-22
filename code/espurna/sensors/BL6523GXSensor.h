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

        void setInverted(bool inverted) {
            if (_inverted == inverted) return;
            _inverted = inverted;
            _dirty = true;
        }

        // ---------------------------------------------------------------------

        unsigned char getRX() {
            return _pin_rx;
        }

        bool getInverted() {
            return _inverted;
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
            if (_serial_is_hardware()) {
                snprintf(buffer, sizeof(buffer), "BL6523GX @ HwSerial");
            } else {
                snprintf(buffer, sizeof(buffer), "BL6523GX @ SwSerial(%u,%u,NULL)", _pin_rx, _pin_tx);
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
            if (index == 6) return _energy[0].asDouble();
            return 0;
        }

    protected:

        // ---------------------------------------------------------------------
        // Protected
        // ---------------------------------------------------------------------

      
        void _process() {

            // Sample data:
            // 55 5A 02 E9 50 00 03 31 00 3E 9E 00 0D 30 4F 44 F8 00 12 65 F1 81 76 72 (w/ load)
            // F2 5A 02 E9 50 00 03 2B 00 3E 9E 02 D7 7C 4F 44 F8 CF A5 5D E1 B3 2A B4 (w/o load)

            #if SENSOR_DEBUG
                DEBUG_MSG("[SENSOR] BL6523GX: _process: ");
                for (byte i=0; i<24; i++) DEBUG_MSG("%02X ", _data[i]);
                DEBUG_MSG("\n");
            #endif

            // Checksum
            if (!_checksum()) {
                _error = SENSOR_ERROR_CRC;
                #if SENSOR_DEBUG
                    DEBUG_MSG("[SENSOR] BL6523GX: Checksum error\n");
                #endif
                return;
            }

            // Calibration
            if (0xAA == _data[0]) {
                _error = SENSOR_ERROR_CALIBRATION;
                #if SENSOR_DEBUG
                    DEBUG_MSG("[SENSOR] BL6523GX: Chip not calibrated\n");
                #endif
                return;
            }

            if ((_data[0] & 0xFC) > 0xF0) {
                _error = SENSOR_ERROR_OTHER;
                #if SENSOR_DEBUG
                    if (0xF1 == (_data[0] & 0xF1)) DEBUG_MSG_P(PSTR("[SENSOR] BL6523GX: Abnormal coefficient storage area\n"));
                    if (0xF2 == (_data[0] & 0xF2)) DEBUG_MSG_P(PSTR("[SENSOR] BL6523GX: Power cycle exceeded range\n"));
                    if (0xF4 == (_data[0] & 0xF4)) DEBUG_MSG_P(PSTR("[SENSOR] BL6523GX: Current cycle exceeded range\n"));
                    if (0xF8 == (_data[0] & 0xF8)) DEBUG_MSG_P(PSTR("[SENSOR] BL6523GX: Voltage cycle exceeded range\n"));
                #endif
                return;
            }

            // Calibration coefficients
            unsigned long _coefV = (_data[2]  << 16 | _data[3]  << 8 | _data[4] );              // 190770
            unsigned long _coefC = (_data[8]  << 16 | _data[9]  << 8 | _data[10]);              // 16030
            unsigned long _coefP = (_data[14] << 16 | _data[15] << 8 | _data[16]);              // 5195000

            // Adj: this looks like a sampling report
            uint8_t adj = _data[20];                                                            // F1 11110001

            // Calculate voltage
            _voltage = 0;
            if ((adj & 0x40) == 0x40) {
                unsigned long voltage_cycle = _data[5] << 16 | _data[6] << 8 | _data[7];        // 817
                _voltage = _voltage_ratio * _coefV / voltage_cycle / BL6523GX_V2R;                      // 190700 / 817 = 233.41
            }

            // Calculate power
            _active = 0;
            if ((adj & 0x10) == 0x10) {
                if ((_data[0] & 0xF2) != 0xF2) {
                    unsigned long power_cycle = _data[17] << 16 | _data[18] << 8 | _data[19];   // 4709
                    _active = _power_active_ratio * _coefP / power_cycle / BL6523GX_V1R / BL6523GX_V2R;       // 5195000 / 4709 = 1103.20
                }
            }

            // Calculate current
            _current = 0;
            if ((adj & 0x20) == 0x20) {
                if (_active > 0) {
                    unsigned long current_cycle = _data[11] << 16 | _data[12] << 8 | _data[13]; // 3376
                    _current = _current_ratio * _coefC / current_cycle / BL6523GX_V1R;                  // 16030 / 3376 = 4.75
                }
            }

            // Calculate reactive power
            _reactive = 0;
            unsigned int active = _active;
            unsigned int apparent = _voltage * _current;
            if (apparent > active) {
                _reactive = sqrt(apparent * apparent - active * active);
            } else {
                _reactive = 0;
            }

            // Calculate energy
            uint32_t cf_pulses = _data[21] << 8 | _data[22];

            static uint32_t cf_pulses_last = 0;
            if (0 == cf_pulses_last) cf_pulses_last = cf_pulses;

            uint32_t difference;
            if (cf_pulses < cf_pulses_last) {
                difference = cf_pulses + (0xFFFF - cf_pulses_last) + 1;
            } else {
                difference = cf_pulses - cf_pulses_last;
            }

            _energy[0] += sensor::Ws {
                static_cast<uint32_t>(difference * (float) _coefP / 1000000.0)
            };
            cf_pulses_last = cf_pulses;

        }

        void _read() {

            _error = SENSOR_ERROR_OK;

            static unsigned char index = 0;
            static unsigned long last = millis();
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

            AddLogBuffer(LOG_LEVEL_DEBUG_MORE, rx_buffer, BL6523_RX_DATASET_SIZE);
            
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
                Energy.current[SINGLE_PHASE] =  (float)((tx_buffer[2] << 16) | (tx_buffer[1] << 8) | tx_buffer[0]) / Settings->energy_current_calibration;     // 1.260 A
                break;
            case BL6523_REG_VOLTS :
                Energy.voltage[SINGLE_PHASE] = (float)((tx_buffer[2] << 16) | (tx_buffer[1] << 8) | tx_buffer[0]) / Settings->energy_voltage_calibration;     // 230.2 V
                break;
            case BL6523_REG_FREQ :
                Energy.frequency[SINGLE_PHASE] = (float)((tx_buffer[2] << 16) | (tx_buffer[1] << 8) | tx_buffer[0]) / Settings->energy_frequency_calibration;    // 50.0 Hz
                break;        
            case BL6523_REG_WATTS :
                Energy.active_power[SINGLE_PHASE] = (float)((tx_buffer[2] << 16) | (tx_buffer[1] << 8) | tx_buffer[0]) / Settings->energy_power_calibration; // -196.3 W
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
            Energy.power_factor[SINGLE_PHASE] = powf;
                break;
            case BL6523_REG_WATTHR :
                Energy.import_active[SINGLE_PHASE] = (float)((tx_buffer[2] << 16) | (tx_buffer[1] << 8) | tx_buffer[0]) / ( Settings->energy_power_calibration - BL6523_PWHRREF_D ); // 6.216 kWh => used in EnergyUpdateTotal()
                break;
            default :  
            break;            
            }
            Energy.data_valid[SINGLE_PHASE] = 0;
            EnergyUpdateTotal();
            if (!Bl6523.discovery_triggered)
            {
                TasmotaGlobal.discovery_counter = 1; // force TasDiscovery()
                Bl6523.discovery_triggered = true;
            }
            return true;
            

            // Process packet
            if (24 == index) {
                _process();
                index = 0;
            }

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

         void _serial_tx_readbytes() {
            if (_serial_tx_is_hardware()) {
                return Serial.readBytes();
            } else {
                return _serial_tx->readBytes();
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
         void _serial_rx_readbytes( ) {
            if (_serial_rx_is_hardware()) {
                return Serial.readBytes();
            } else {
                return _serial_rx->readBytes();
            }
        }       

        // ---------------------------------------------------------------------

        int _pin_rx = BL6523GX_RX_PIN;
        int _pin_tx = BL6523GX_TX_PIN;
        bool _inverted_rx = BL6523GX_RX_PIN_INVERSE;
        bool _inverted_tx = BL6523GX_RX_PIN_INVERSE;
        std::unique_ptr<SoftwareSerial> _serial_tx;
        std::unique_ptr<SoftwareSerial> _serial_rx;

        double _active = 0;
        double _reactive = 0;
        double _voltage = 0;
        double _current = 0;

        unsigned char _data[24] {0};

};

#if __cplusplus < 201703L
constexpr BaseEmonSensor::Magnitude BL6523GXSensor::Magnitudes[];
#endif

#endif // SENSOR_SUPPORT && BL6523GX_SUPPORT
