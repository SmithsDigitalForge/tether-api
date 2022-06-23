//
// Created by sbasu on 8/31/21.
// Based on the FALCON API (in the ok/rust/rusthdl repo).
//

#ifndef RUSTHDL_TETHER_ICE9_API_H
#define RUSTHDL_TETHER_ICE9_API_H

#ifdef __cplusplus
#define EXTERN_C extern "C"
#else  // ! __cplusplus
#define EXTERN_C
#endif  // __cplusplus

#include <stdint.h>


enum TetherError {
    Tether_OK,
    Tether_Error,
    Tether_DeviceInitializationFailed,
    Tether_CommunicationError,
    Tether_HandleInvalid,
    Tether_DataStreamMismatch,
    Tether_Timeout,
    Tether_ProtocolError,
    Tether_BitfileDownloadFailed,
    Tether_InvalidParameter
};

EXTERN_C const char * tether_error_string(enum TetherError ecode);

EXTERN_C struct tether_handle * tether_handle_new();

EXTERN_C void tether_handle_free(struct tether_handle *hnd);

EXTERN_C enum TetherError tether_load_firmware(struct tether_handle *hnd, const char *filename, int simulated_mode);

EXTERN_C enum TetherError tether_initialize_firmware(struct tether_handle *hnd);

EXTERN_C uint32_t get_firmware_base_clock_frequency(struct tether_handle *hnd);

/*
 * Reset the firmware - this is required as the first step
 * after the firmware is loaded.  It forces the Memory interface
 * to go through its reset cycle.  Currently, it takes 0.1 second,
 * but if needed, it could be significantly shortened.
 */
EXTERN_C enum TetherError reset_firmware(struct tether_handle *hnd);

/*
 * Configure the firmware for either a DGS or Pump config
 * Currently known configs:
 *    0 - Pump config (4x4-20, 8-TC, 3xHF)
 *    1 - DGS config (4x4-20, 2xLPS, 4xtach, 3xHF)
 */
#define TUNI_PUMP 0
#define TUNI_DGS 1
#define TUNI_UNKNOWN 0xFFFF
EXTERN_C enum TetherError set_firmware_config(struct tether_handle * hnd, uint16_t config);

/*
 * Select an ADC on the 4-20/LPS board. chain  Note that this assumes that
 * the SPI master is actually enabled.  If the TETHER-DAQ is running
 * it does nothing.
 */
EXTERN_C enum TetherError select_adc_on_LF_board(struct tether_handle * hnd, int8_t address);

/*
 * Select a TC chip on the TC board chain.
 */
EXTERN_C enum TetherError select_tc_on_LF_board(struct tether_handle * hnd, int8_t address);

/*
 * Select an ADC on the MF board
 */
EXTERN_C enum TetherError select_adc_on_MF_board(struct tether_handle * hnd, int8_t address);

/*
 * Perform a SPI transaction on the LF bus, using the SPI master
 * on the FPGA.  The SPI master must be enabled for this to have
 * and effect.  The transaction is directed at the adc selected
 * by the select_adc_on_LF_board function.
 */
EXTERN_C enum TetherError do_spi_txn_lf(struct tether_handle * hnd, uint8_t bits, uint64_t mosi, uint64_t *miso);

/*
 * Perform a SPI transaction on the LF bus, using the SPI master
 * on the FPGA.  The SPI master must be enabled for this to have
 * and effect.  The transaction is directed at the adc selected
 * by the select_adc_on_LF_board function.
 */
EXTERN_C enum TetherError do_spi_txn_lf_mode(struct tether_handle * hnd, uint8_t bits, uint64_t mosi, uint64_t *miso, uint8_t mode);

/*
 * Perform a SPI transaction on the MF bus, using the SPI master
 * on the FPGA.  The SPI master must be enabled for this to have
 * and effect.  The transaction is directed at the adc selected
 * by the select_adc_on_MF_board function.
 */
EXTERN_C enum TetherError do_spi_txn_mf(struct tether_handle * hnd, uint8_t bits, uint64_t mosi, uint64_t *miso);

/*
 * Write a word to an ADC register - assumes we are writing to the ADS868X.  Targets the
 * currently selected address.
 */
EXTERN_C enum TetherError write_adc_register(struct tether_handle * hnd, uint8_t reg, uint16_t value);

/*
 * Write a word to an ADC register on an MF board
 */
EXTERN_C enum TetherError write_mf_register(struct tether_handle * hnd, uint8_t reg, uint16_t value);

/*
 * Read a word from an ADC register - assumes we are reading from the ADS868X.  Targets the
 * currently selected address
 */
EXTERN_C enum TetherError read_adc_register_word(struct tether_handle * hnd, uint8_t reg, uint16_t *value);

/*
 * Read a byte from an ADC register - assumes we are reading from the ADS868X.  Targets the
 * currently selected address
 */
EXTERN_C enum TetherError read_adc_register_byte(struct tether_handle * hnd, uint8_t reg, uint8_t *value);

/*
 * Read a word from an ADC register on an MF board
 */
EXTERN_C enum TetherError read_mf_register_word(struct tether_handle * hnd, uint8_t reg, uint16_t *value);

/*
 * Read a byte from an ADC register on an MF board
 */
EXTERN_C enum TetherError read_mf_register_byte(struct tether_handle * hnd, uint8_t reg, uint8_t *value);

/*
 * Read a byte form a TC register - assumes we are reading from the MAX31856 chip.  Targets
 * the currently selected address
 */
EXTERN_C enum TetherError read_tc_register_byte(struct tether_handle * hnd, uint8_t reg, uint8_t *value);

/*
 * Write a byte to a TC register - assumes we are writing to the MAX31856 chip.  Targets
 * the currently selected address
 */
EXTERN_C enum TetherError write_tc_register_byte(struct tether_handle * hnd, uint8_t reg, uint8_t value);

/*
 * Enable/Disable the LF DAQ.  If the LF DAQ is enabled, the SPI master is automatically disabled.
 * If the LF DAQ is disabled, the SPI Master is enabled.
 */
EXTERN_C enum TetherError control_lf_daq(struct tether_handle * hnd, uint8_t flag);

/*
 * Enable/Disable the HF DAQ.  IF the HF DAQ is enabled, it starts collecting data automatically.
 */
EXTERN_C enum TetherError control_hf_daq(struct tether_handle * hnd, uint8_t flag);

/*
 * Enable/Disable the MF DAQ.  If the MF DAQ is enabled, it starts collecting data automatically.
 */
EXTERN_C enum TetherError control_mf_daq(struct tether_handle * hnd, uint8_t flag);

/*
 * Read a block of data from the download FIFO.  This will block if insufficient data is available,
 * and timeout if the data is ultimately not produced.
 */
EXTERN_C enum TetherError read_data_from_download_fifo(struct tether_handle * hnd, uint32_t bytes_to_read, uint8_t *data);

/*
 *  Set the LFDAQ Round time.  This is the duration between the start of channel sampling on the LF-DAQ.
 *  If it is zero (default), then the LF-DAQ will run as fast as possible.  If you set it to, e.g. 100,
 *  then the start of each DAQ round will be separated by 100 clock cycles.  Note that if the delay
 *  is less than the time to complete all of the samples (nominally 1 msec * num_configurations)
 *  then this does nothing.
 */
EXTERN_C enum TetherError set_lfdaq_round_time_clocks(struct tether_handle * hnd, uint32_t time_in_clocks);

/*
 * Set the TC decimation factor.  This is the number of data rounds that are collected of the non-TC
 * channels for each round of TC data.
 */
EXTERN_C enum TetherError set_lfdaq_tc_decimation_factor(struct tether_handle * hnd, uint16_t decimation_factor);

/*
 * Set the MFDAQ round time. Note that the MF channels are all sampled simultaneously.  So we need
 * this time to exceed the conversion time for a channel and the time to read the channel out.
 * The time is measured in clocks between samples.  The clock is 48MHz.
 */
EXTERN_C enum TetherError set_mfdaq_round_time_clocks(struct tether_handle * hnd, uint32_t time_in_clocks);

EXTERN_C enum TetherError get_download_fifo_status_register(struct tether_handle * hnd, uint16_t *status);

EXTERN_C enum TetherError get_cdsf_firmware_version(struct tether_handle * hnd, uint16_t *major, uint16_t *minor, uint16_t *patch);

EXTERN_C enum TetherError ping_bridge(struct tether_handle *, uint8_t pingid);

EXTERN_C double get_highres_time();

EXTERN_C enum TetherError do_spi_txn(struct tether_handle * hnd, uint8_t base_address, uint16_t bits, uint64_t mosi, uint64_t *miso);

#endif //RUSTHDL_TETHER_ICE9_API_H
