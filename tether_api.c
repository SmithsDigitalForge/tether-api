#pragma clang diagnostic push
#pragma ide diagnostic ignored "EmptyDeclOrStmt"
//
// Created by sbasu on 4/8/20.
//

#include "tether_api.h"
#include "logger.h"
#include <unistd.h>
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <bits/types/struct_timespec.h>
#include <time.h>
#include "ice9.h"
#include "miniz.h"
#include <okFrontPanel.h>

#define lib_try(x) {ecode = (x); if (ecode != 0) {return(ecode);}}


const uint8_t download_reset = 0;
const uint8_t download_data = 1;
const uint8_t lf_dgs_master_base = 2;
const uint8_t lf_dgs_mux_select = 6;
const uint8_t lf_dgs_control_daq_enable = 7;
const uint8_t lf_dgs_control_adc_select = 8;
const uint8_t lf_dgs_control_inter_sample_delay = 9;
const uint8_t lf_dgs_control_tc_decimation = 10;
const uint8_t lf_pump_master_base = 11;
const uint8_t lf_pump_mux_select = 15;
const uint8_t lf_pump_control_daq_enable = 16;
const uint8_t lf_pump_control_adc_select = 17;
const uint8_t lf_pump_control_inter_sample_delay = 18;
const uint8_t lf_pump_control_tc_decimation = 19;
const uint8_t hf_enable = 20;
const uint8_t tach_master_base = 21;
const uint8_t tach_control_daq_spi_select = 25;
const uint8_t tach_control_daq_enable = 26;
const uint8_t tach_control_adc_select = 27;
const uint8_t tach_control_inter_sample_delay = 28;
const uint8_t control_config = 29;
const uint8_t control_version = 30;
const uint8_t cjt_enable = 31;
const uint8_t accel_master_base = 32;
const uint8_t accel_mux_select = 36;
const uint8_t accel_control_daq_enable = 37;
const uint8_t accel_control_inter_sample_delay = 38;


static enum TetherError ecode;

static uint16_t my_config = TUNI_UNKNOWN;
static uint8_t streaming_mode = 0;

const uint8_t config_space_avail = 0x3E;
const uint8_t config_pipe_in = 0x9D;
const uint8_t config_pipe_out = 0xBD;
const uint8_t config_words_avail = 0x3D;
const uint8_t config_block_flow_control = 0x1D;


struct tether_handle {
  okFrontPanel_HANDLE ok;
  struct ice9_handle *ice9;
  int is_ice9;
  int is_valid;
};

struct tether_handle * tether_handle_new() {
    struct tether_handle *hnd = calloc(1, sizeof(struct tether_handle));
    hnd->ok = okFrontPanel_Construct();
    hnd->is_valid = 1;
    if (okFrontPanel_GetDeviceCount(hnd->ok) == 0) {
        // No OpalKelly devices... assume ice9
        LOG_INFO("No opal kelly devices found.. Assuming ice9\n");
        okFrontPanel_Destruct(hnd->ok);
        hnd->is_ice9 = 1;
        hnd->ice9 = ice9_new();
    } else {
        LOG_INFO("Found an opal kelly device.  Assuming this is an OK setup\n");
    }
    return hnd;
}

void tether_handle_free(struct tether_handle *hnd) {
    if (hnd) {
        if (hnd->is_valid) {
            if (hnd->is_ice9) {
                ice9_close(hnd->ice9);
                ice9_free(hnd->ice9);
            } else {
                okFrontPanel_Close(hnd->ok);
                okFrontPanel_Destruct(hnd->ok);
            }
        }
        free(hnd);
    }
}



const char * tether_error_string(enum TetherError ecode) {
    switch (ecode) {
        case Tether_OK:
            return "No Error";
        case Tether_Error:
            return "Generic Error";
        case Tether_DeviceInitializationFailed:
            return "Device Initialization Failed";
        case Tether_CommunicationError:
            return "Communication Error";
        case Tether_HandleInvalid:
            return "Invalid Handle";
        case Tether_DataStreamMismatch:
            return "Data Stream Mismatch";
        case Tether_Timeout:
            return "Timeout";
        case Tether_ProtocolError:
            return "Protocol Error";
        case Tether_BitfileDownloadFailed:
            return "Bit File Download Failed";
        case Tether_InvalidParameter:
            return "Invalid Parameter";
        default:
            return "Unknown Error";
    }
}

enum TetherError map_ok_error_code_to_tether(ok_ErrorCode ecode) {
    switch (ecode)
    {
        case ok_NoError:
            return Tether_OK;
        default:
            return Tether_Error;
    }
}

enum TetherError map_ice9_error_code_to_tether(enum Ice9Error ecode) {
    switch (ecode)
    {
        case OK:
            return Tether_OK;
        case Error:
            return Tether_Error;
        case LibUSBTimeout:
            return Tether_Timeout;
        default:
            return Tether_CommunicationError;
    }
}

enum TetherError tether_load_firmware(struct tether_handle *hnd, const char * filename, int simulated_mode) {
    if (!hnd || !hnd->is_valid) {
        return Tether_HandleInvalid;
    }
    mz_zip_archive zip_archive;
    void *buf = NULL;
    size_t bufsize;

    memset(&zip_archive, 0, sizeof(mz_zip_archive));
    
    if (!mz_zip_reader_init_file(&zip_archive, filename, 0)) {
        return Tether_BitfileDownloadFailed;
    }

    const char *fname = NULL;
    if (hnd->is_ice9 && simulated_mode) {
        fname = "ice9_sim.bit";
    }
    if (hnd->is_ice9 && !simulated_mode) {
        fname = "ice9_hw.bit";
    }
    if (!hnd->is_ice9 && simulated_mode) {
        fname = "ok_sim.bit";
    }
    if (!hnd->is_ice9 && !simulated_mode) {
        fname = "ok_hw.bit";
    }
    if (!fname) {
        return Tether_InvalidParameter;
    }
    LOG_INFO("Selected firmware %s\n", fname);

    buf = mz_zip_reader_extract_file_to_heap(&zip_archive, fname, &bufsize, 0);

    if (!buf) {
        return Tether_BitfileDownloadFailed;
    }
    LOG_INFO("Read firmware buffer of size %zu\n", bufsize);
    mz_zip_reader_end(&zip_archive);
    
    if (hnd->is_ice9) {
        ice9_flash_fpga_mem(buf, bufsize);
    } else {
        lib_try(map_ok_error_code_to_tether(okFrontPanel_OpenBySerial(hnd->ok, NULL)));
        lib_try(map_ok_error_code_to_tether(okFrontPanel_ConfigureFPGAFromMemory(hnd->ok, buf, bufsize)));
    }
    free(buf);
    return Tether_OK;
}

enum TetherError tether_initialize_firmware(struct tether_handle *hnd) {
    if (!hnd || !hnd->is_valid) {
        return Tether_HandleInvalid;
    }
    if (hnd->is_ice9) {
        lib_try(map_ice9_error_code_to_tether(ice9_open(hnd->ice9)));
        lib_try(map_ice9_error_code_to_tether(ice9_usb_reset(hnd->ice9)));
        lib_try(map_ice9_error_code_to_tether(ice9_fifo_mode(hnd->ice9)));
    }
    return Tether_OK;
}


enum TetherError write_bridge_bytes_ok(okFrontPanel_HANDLE hnd, uint8_t *data, uint16_t len) {
    if (len % 2 != 0) {
        return Tether_InvalidParameter;
    }
    uint8_t send_ok = FALSE;
    for (int retry=0;retry < 100;retry++) {
        lib_try(okFrontPanel_UpdateWireOuts(hnd));
        if (okFrontPanel_GetWireOutValue(hnd, config_space_avail) >= (len / 2)) {
            send_ok = TRUE;
            break;
        }
        usleep(5000);
    }
    if (!send_ok) {
        return Tether_Timeout;
    }
    long ret = okFrontPanel_WriteToPipeIn(hnd, config_pipe_in, len, data);
    if (ret < 0) {
        return Tether_CommunicationError;
    } else {
        return Tether_OK;
    }
}

enum TetherError read_bridge_bytes_ok(okFrontPanel_HANDLE hnd, uint8_t *data, uint16_t len) {
    if (len % 2 != 0) {
        return Tether_InvalidParameter;
    }
    uint8_t send_ok = FALSE;
    for (int retry=0;retry < 100; retry++) {
        lib_try(okFrontPanel_UpdateWireOuts(hnd));
        if (okFrontPanel_GetWireOutValue(hnd, config_words_avail) >= (len/2)) {
            send_ok = TRUE;
            break;
        }
        usleep(5000);
    }
    if (!send_ok) {
        return Tether_Timeout;
    }
    long res = okFrontPanel_ReadFromPipeOut(hnd, config_pipe_out, len, data);
    if (res < 0) {
        return Tether_CommunicationError;
    }
    return Tether_OK;
}

enum TetherError write_bridge_bytes_ice9(struct ice9_handle * hnd, uint8_t *data, uint16_t len) {
    return map_ice9_error_code_to_tether(ice9_write(hnd, data, len));
}

enum TetherError read_bridge_bytes_ice9(struct ice9_handle * hnd, uint8_t *data, uint16_t len) {
    return map_ice9_error_code_to_tether(ice9_read(hnd, data, len));
}

enum TetherError write_bridge_bytes(struct tether_handle * hnd, uint8_t *data, uint16_t len) {
    if (!hnd || !hnd->is_valid) {
        return Tether_HandleInvalid;
    }
    if (hnd->is_ice9) {
        return write_bridge_bytes_ice9(hnd->ice9, data, len);
    } else {
        return write_bridge_bytes_ok(hnd->ok, data, len);
    }
}

enum TetherError read_bridge_bytes(struct tether_handle * hnd, uint8_t *data, uint16_t len) {
    if (!hnd || !hnd->is_valid) {
        return Tether_HandleInvalid;
    }
    if (hnd->is_ice9) {
        return read_bridge_bytes_ice9(hnd->ice9, data, len);
    } else {
        return read_bridge_bytes_ok(hnd->ok, data, len);
    }
}

enum TetherError write_bridge_words(struct tether_handle * hnd, uint16_t* values, uint16_t len) {
    return write_bridge_bytes(hnd, (uint8_t*)(values), len * 2);
}

enum TetherError write_bridge_word(struct tether_handle * hnd, uint16_t value) {
    return write_bridge_bytes(hnd, (uint8_t*)(&value), 2);
}

enum TetherError read_bridge_words(struct tether_handle * hnd, uint16_t *values, uint16_t len) {
    return read_bridge_bytes(hnd, (uint8_t*)(values), len*2);
}

enum TetherError read_bridge_word(struct tether_handle * hnd, uint16_t *ptr) {
    if (ptr) {
        return read_bridge_bytes(hnd, (uint8_t*)(ptr), 2);
    }
    return Tether_Error;
}

enum TetherError write_data_to_address(struct tether_handle * hnd, uint8_t address, uint16_t *values, uint16_t len) {
    uint16_t header[2];
    header[0] = 0x0300 | address;
    header[1] = len;
    lib_try(write_bridge_words(hnd, header, 2));
    return write_bridge_words(hnd, values, len);
}

enum TetherError write_word_to_address(struct tether_handle * hnd, uint8_t address, uint16_t value) {
    return write_data_to_address(hnd, address, &value, 1);
}

enum TetherError write_int_to_address(struct tether_handle * hnd, uint8_t address, uint32_t value) {
    uint16_t vals[2];
    vals[0] = (value >> 16) & 0xFFFF;
    vals[1] = value & 0xFFFF;
    return write_data_to_address(hnd, address, vals, 2);
}

enum TetherError read_data_from_address(struct tether_handle * hnd, uint8_t address, uint16_t *values, uint16_t len) {
    uint16_t header[2];
    header[0] = 0x0200 | address;
    header[1] = len;
    lib_try(write_bridge_words(hnd, header, 2));
    return read_bridge_words(hnd, values, len);
}

enum TetherError send_bridge_ping(struct tether_handle * hnd, uint8_t pingid) {
    return(write_bridge_word(hnd, 0x0100 | pingid));
}

enum TetherError ping_bridge(struct tether_handle * hnd, uint8_t pingid) {
    lib_try(send_bridge_ping(hnd, pingid));
    uint16_t pingret = 0;
    lib_try(read_bridge_word(hnd, &pingret));
    pingret = pingret & 0xFF;
    if (pingret != pingid) {
        return Tether_DataStreamMismatch;
    }
    return Tether_OK;
}


enum TetherError reset_firmware(struct tether_handle * hnd) {
    lib_try(write_word_to_address(hnd, download_reset, 1));
    usleep(100*1000);
    return write_word_to_address(hnd, download_reset, 0);
}

enum TetherError set_firmware_config(struct tether_handle * hnd, uint16_t config) {
    if (config > 1)
        return Tether_InvalidParameter;
    my_config = config;
    switch (config) {
        case TUNI_DGS:
            return write_word_to_address(hnd, control_config, 0);
        case TUNI_PUMP:
            return write_word_to_address(hnd, control_config, 1);
        default:
            return Tether_Error;
    }
}

enum TetherError select_adc_on_LF_board(struct tether_handle * hnd, int8_t address) {
    switch (my_config) {
        case TUNI_PUMP:
            return write_word_to_address(hnd, lf_pump_control_adc_select, address);
        case TUNI_DGS:
            return write_word_to_address(hnd, lf_dgs_control_adc_select, address);
        default:
            return Tether_Error;
    }
}

// TC board is pump only (for now)
enum TetherError select_tc_on_LF_board(struct tether_handle * hnd, int8_t address) {
    return write_word_to_address(hnd, lf_pump_control_adc_select, address + 8);
}

// MF board is DGS only (for now)
enum TetherError select_adc_on_MF_board(struct tether_handle * hnd, int8_t address) {
    return write_word_to_address(hnd, tach_control_adc_select, address);
}

enum TetherError do_spi_txn(struct tether_handle * hnd, uint8_t base_address, uint16_t bits, uint64_t mosi, uint64_t *miso) {
    uint16_t data[2];
    assert((bits & 0xFF) <= 32);
    data[0] = (mosi >> 16) & 0xFFFF;
    data[1] = (mosi & 0xFFFF);
    lib_try(write_data_to_address(hnd, base_address, data, 2));
    lib_try(write_word_to_address(hnd, base_address + 2, bits));
    lib_try(write_word_to_address(hnd, base_address + 3, 0));
    lib_try(read_data_from_address(hnd, base_address + 1, data, 2));
    if (miso) {
        *miso = (((uint64_t) (data[0])) << 16) | (((uint64_t) (data[1])));
    }
    return Tether_OK;
}

enum TetherError do_spi_txn_lf(struct tether_handle * hnd, uint8_t bits, uint64_t mosi, uint64_t *miso) {
    return do_spi_txn_lf_mode(hnd, bits, mosi, miso, 0);
}

enum TetherError do_spi_txn_lf_mode(struct tether_handle * hnd, uint8_t bits, uint64_t mosi, uint64_t *miso, uint8_t mode) {
    switch (my_config) {
        case TUNI_PUMP:
            return do_spi_txn(hnd, lf_pump_master_base, bits | (mode << 8), mosi, miso);
        case TUNI_DGS:
            return do_spi_txn(hnd, lf_dgs_master_base, bits | (mode << 8), mosi, miso);
        default:
            return Tether_Error;
    }
}

enum TetherError read_tc_register_byte(struct tether_handle * hnd, uint8_t reg, uint8_t *reg_value) {
    uint64_t cmd = ((uint64_t) reg) << 8;
    uint64_t result;
    lib_try(do_spi_txn_lf_mode(hnd, 16, cmd, &result, 3));
    if (!reg_value) {
        return Tether_OK;
    }
    *reg_value = result & 0xff;
    return Tether_OK;
}

enum TetherError write_tc_register_byte(struct tether_handle * hnd, uint8_t reg, uint8_t reg_value) {
    uint64_t cmd = (((uint64_t) ((1 << 7) | reg)) << 8) | reg_value;
    uint64_t result;
    lib_try(do_spi_txn_lf_mode(hnd, 16, cmd, &result, 3));
    return Tether_OK;
}

enum TetherError do_spi_txn_mf(struct tether_handle * hnd, uint8_t bits, uint64_t mosi, uint64_t* miso) {
    switch (my_config) {
        case TUNI_PUMP:
            return do_spi_txn(hnd, accel_master_base, bits, mosi, miso);
        case TUNI_DGS:
            return do_spi_txn(hnd, tach_master_base, bits, mosi, miso);
        default:
            return Tether_Error;
    }
}

enum TetherError read_mf_register_byte_dgs(struct tether_handle * hnd, uint8_t reg, uint8_t *reg_value) {
    uint64_t cmd = 0x48000000 | (((uint32_t) reg) << 16);
    uint64_t result;
    lib_try(do_spi_txn_mf(hnd,32, cmd, &result));
    if (!reg_value) {
        return Tether_OK;
    }
    cmd = 0x0;
    lib_try(do_spi_txn_mf(hnd, 8, cmd, &result));
    *reg_value = (uint8_t) (result & 0xFF);
    return Tether_OK;
}

enum TetherError read_mf_register_byte_pump(struct tether_handle * hnd, uint8_t reg, uint8_t *reg_value) {
    uint64_t cmd = ((uint32_t) reg) << 17;
    uint64_t result;
    lib_try(do_spi_txn_mf(hnd,24, cmd, &result));
    if (!reg_value) {
        return Tether_OK;
    }
    *reg_value = (uint8_t) (result & 0xFF);
    return Tether_OK;
}

enum TetherError read_mf_register_byte(struct tether_handle * hnd, uint8_t reg, uint8_t *reg_value) {
    switch (my_config) {
        case TUNI_DGS:
            return read_mf_register_byte_dgs(hnd, reg, reg_value);
        case TUNI_PUMP:
            return read_mf_register_byte_pump(hnd, reg, reg_value);
        default:
            return Tether_Error;
    }
}

enum TetherError read_adc_register_byte(struct tether_handle * hnd, uint8_t reg, uint8_t *reg_value) {
    uint64_t cmd = 0x48000000 | (((uint32_t) reg) << 16);
    uint64_t result;
    lib_try(do_spi_txn_lf(hnd, 32, cmd, &result));
    if (!reg_value) {
        return Tether_OK;
    }
    cmd = 0x0;
    lib_try(do_spi_txn_lf(hnd, 8, cmd, &result));
    *reg_value = (uint8_t) (result & 0xFF);
    return Tether_OK;
}

enum TetherError read_adc_register_word(struct tether_handle * hnd, uint8_t reg, uint16_t *reg_value) {
    uint64_t cmd = 0xC8000000 | (((uint32_t) reg) << 16);
    uint64_t result;
    lib_try(do_spi_txn_lf(hnd, 32, cmd, &result));
    if (!reg_value) {
        return Tether_OK;
    }
    cmd = 0x0;
    lib_try(do_spi_txn_lf(hnd, 16, cmd, &result));
    *reg_value = (uint16_t) (result & 0xFFFF);
    return Tether_OK;
}

enum TetherError write_adc_register(struct tether_handle * hnd, uint8_t reg, uint16_t reg_value) {
    uint64_t cmd = 0xD0000000 | (((uint32_t) reg) << 16) | (reg_value);
    return do_spi_txn_lf(hnd, 32, cmd, NULL);
}

enum TetherError read_mf_register_word(struct tether_handle * hnd, uint8_t reg, uint16_t *reg_value) {
    uint64_t cmd = 0xC8000000 | (((uint32_t) reg) << 16);
    uint64_t result;
    lib_try(do_spi_txn_mf(hnd, 32, cmd, &result));
    if (!reg_value) {
        return Tether_OK;
    }
    cmd = 0x0;
    lib_try(do_spi_txn_mf(hnd, 16, cmd, &result));
    *reg_value = (uint16_t) (result & 0xFFFF);
    return Tether_OK;
}

enum TetherError write_mf_register(struct tether_handle * hnd, uint8_t reg, uint16_t reg_value) {
    switch (my_config) {
        case TUNI_DGS: {
            uint64_t cmd = 0xD0000000 | (((uint32_t) reg) << 16) | (reg_value);
            return do_spi_txn_mf(hnd, 32, cmd, NULL);
        }
        case TUNI_PUMP: {
            uint64_t cmd = (((uint32_t) reg) << 17) | (1 << 16) | (reg_value << 8);
            return do_spi_txn_mf(hnd, 24, cmd, NULL);
        }
        default:
            return Tether_Error;
    }
}

enum TetherError control_mf_daq(struct tether_handle * hnd, uint8_t flag) {
    switch (my_config) {
        case TUNI_PUMP:
        {
            if (flag != 0) {
                lib_try(write_word_to_address(hnd, accel_mux_select, 1));
                return write_word_to_address(hnd, accel_control_daq_enable, 1);
            } else {
                lib_try(write_word_to_address(hnd, accel_mux_select, 0));
                return write_word_to_address(hnd, accel_control_daq_enable, 0);
            }
        }
        case TUNI_DGS:
        {
            if (flag != 0) {
                lib_try(write_word_to_address(hnd, tach_control_daq_spi_select, 1));
                return write_word_to_address(hnd, tach_control_daq_enable, 1);
            } else {
                lib_try(write_word_to_address(hnd, tach_control_daq_enable, 0));
                return write_word_to_address(hnd, tach_control_daq_spi_select, 0);
            }
        }
        default:
            return Tether_Error;
    }
}

enum TetherError control_dgs_lf_daq(struct tether_handle * hnd, uint8_t flag) {
    lib_try(write_word_to_address(hnd, lf_dgs_mux_select, flag));
    return write_word_to_address(hnd, lf_dgs_control_daq_enable, flag);
}

enum TetherError control_pump_lf_daq(struct tether_handle * hnd, uint8_t flag) {
    lib_try(write_word_to_address(hnd, cjt_enable, flag));
    lib_try(write_word_to_address(hnd, lf_pump_mux_select, flag));
    return write_word_to_address(hnd, lf_pump_control_daq_enable, flag);
}

enum TetherError set_download_flow_control(okFrontPanel_HANDLE hnd, uint8_t flag) {
    lib_try(okFrontPanel_SetWireInValue(hnd, config_block_flow_control, flag, 0xFFFF));
    return map_ok_error_code_to_tether(okFrontPanel_UpdateWireIns(hnd));
}

enum TetherError enable_streaming(struct tether_handle *hnd) {
    if (!hnd || !hnd->is_valid) {
        return Tether_HandleInvalid;
    }
    if (!streaming_mode) {
        if (!hnd->is_ice9) {
            lib_try(set_download_flow_control(hnd->ok, 1));
        }
        streaming_mode = TRUE;
        return write_bridge_word(hnd, 0x0500 | download_data);
    }
    return Tether_OK;
}

enum TetherError disable_streaming(struct tether_handle * hnd) {
    if (!hnd || !hnd->is_valid) {
        return Tether_HandleInvalid;
    }
    if (streaming_mode) {
        if (!hnd->is_ice9) {
            lib_try(set_download_flow_control(hnd->ok, 0));
        }
        streaming_mode = FALSE;
        return write_bridge_word(hnd, 0xFFFF);
    }
    return Tether_OK;
}

enum TetherError control_lf_daq(struct tether_handle * hnd, uint8_t flag) {
    lib_try(disable_streaming(hnd));
    switch (my_config) {
        case TUNI_PUMP:
            return control_pump_lf_daq(hnd, flag);
        case TUNI_DGS:
            return control_dgs_lf_daq(hnd, flag);
        default:
            return Tether_Error;
    }
}

enum TetherError control_hf_daq(struct tether_handle * hnd, uint8_t flag) {
    lib_try(disable_streaming(hnd));
    return write_word_to_address(hnd, hf_enable, flag);
}

enum TetherError read_data_from_download_fifo_ok(okFrontPanel_HANDLE hnd, uint32_t bytes_to_read, uint8_t *data) {
    long read = okFrontPanel_ReadFromBlockPipeOut(hnd, config_pipe_out, 1024, bytes_to_read, data);
    if (read < 0) {
        return map_ok_error_code_to_tether(read);
    }
    return Tether_OK;
}


enum TetherError read_data_from_download_fifo_ice9(struct ice9_handle * hnd, uint32_t bytes_to_read, uint8_t *data) {
    return map_ice9_error_code_to_tether(ice9_stream_read(hnd, data, bytes_to_read));
}

enum TetherError read_data_from_download_fifo(struct tether_handle *hnd, uint32_t bytes_to_read, uint8_t *data) {
    if (!hnd || !hnd->is_valid) {
        return Tether_HandleInvalid;
    }
    lib_try(enable_streaming(hnd));
    if (hnd->is_ice9) {
        return read_data_from_download_fifo_ice9(hnd->ice9, bytes_to_read, data);
    } else {
        return read_data_from_download_fifo_ok(hnd->ok, bytes_to_read, data);
    }
}

enum TetherError set_dgs_lfdaq_round_time_clocks(struct tether_handle * hnd, uint32_t time_in_clocks) {
    return write_int_to_address(hnd, lf_dgs_control_inter_sample_delay, time_in_clocks);
}

enum TetherError set_pump_lfdaq_round_time_clocks(struct tether_handle * hnd, uint32_t time_in_clocks) {
    return write_int_to_address(hnd, lf_pump_control_inter_sample_delay, time_in_clocks);
}

enum TetherError set_lfdaq_round_time_clocks(struct tether_handle * hnd, uint32_t time_in_clocks) {
    switch (my_config) {
        case TUNI_PUMP: return set_pump_lfdaq_round_time_clocks(hnd, time_in_clocks);
        case TUNI_DGS: return set_dgs_lfdaq_round_time_clocks(hnd, time_in_clocks);
        default:
            return Tether_Error;
    }
}

enum TetherError set_pump_tc_decimation_factor(struct tether_handle * hnd, uint16_t decimation_factor) {
    return write_word_to_address(hnd, lf_pump_control_tc_decimation, decimation_factor);
}

enum TetherError set_dgs_tc_decimation_factor(struct tether_handle * hnd, uint16_t decimation_factor) {
    return write_word_to_address(hnd, lf_dgs_control_tc_decimation, decimation_factor);
}


enum TetherError set_lfdaq_tc_decimation_factor(struct tether_handle * hnd, uint16_t decimation_factor) {
    switch (my_config) {
        case TUNI_PUMP: return set_pump_tc_decimation_factor(hnd, decimation_factor);
        case TUNI_DGS: return set_dgs_tc_decimation_factor(hnd, decimation_factor);
    }
    return Tether_Error;
}

enum TetherError set_mfdaq_round_time_clocks(struct tether_handle * hnd, uint32_t time_in_clocks) {
    switch (my_config) {
        case TUNI_DGS: return write_int_to_address(hnd, tach_control_inter_sample_delay, time_in_clocks);
        case TUNI_PUMP: return write_int_to_address(hnd, accel_control_inter_sample_delay, time_in_clocks);
    }
    return Tether_Error;
}

enum TetherError get_download_fifo_status_register(struct tether_handle * hnd, uint16_t* status) {
    *status = 0;
    return Tether_OK;
}

enum TetherError get_cdsf_firmware_version(struct tether_handle * hnd, uint16_t *major, uint16_t*minor, uint16_t *patch) {
    uint16_t version = 0;
    lib_try(read_data_from_address(hnd, control_version, &version, 1));
    *major = (version >> 12) & 0xF;
    *minor = (version >> 4) & 0xFF;
    *patch = (version) & 0xF;
    return Tether_OK;
}

double get_highres_time() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return (ts.tv_sec * 1e9 + ts.tv_nsec);
}


uint32_t get_firmware_base_clock_frequency(struct tether_handle *hnd) {
    if (!hnd || !hnd->is_valid) {
        return 0;
    }
    if (hnd->is_ice9) {
        return 50000000;
    } else {
        return 48000000;
    }
}


#pragma clang diagnostic pop
