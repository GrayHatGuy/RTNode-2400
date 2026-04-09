// Copyright Sandeep Mistry, Mark Qvist and Jacob Eva.
// Licensed under the MIT license.


#include "Boards.h"

#if MODEM == LR1121

#include "lr1121.h"

#if MCU_VARIANT == MCU_ESP32
  #if MCU_VARIANT == MCU_ESP32 and !defined(CONFIG_IDF_TARGET_ESP32S3)
    #include "soc/rtc_wdt.h"
  #endif
  #define ISR_VECT IRAM_ATTR
#else
  #define ISR_VECT
#endif

// ============================================================
// LR1121 SPI command opcodes (16-bit, MSB first)
// Source: LR1121 datasheet LR1121_v1_2.pdf  
// ============================================================

// --- System group (0x01xx) ---
#define OP_GET_STATUS_11X           0x0100  //o
#define OP_GET_VERSION_11X          0x0101  //o
#define OP_WRITE_REGMEM32_11X       0x0105  //o Write 32-bit aligned registers
#define OP_READ_REGMEM32_11X        0x0106  //o Read  32-bit aligned registers
#define OP_WRITE_BUFFER8_11X        0x0109  //o Write TX FIFO (byte-addressed)
#define OP_READ_BUFFER8_11X         0x010A  //o Read  RX FIFO (byte-addressed)
#define OP_GET_ERRORS_11X           0x010D  //o
#define OP_CLEAR_ERRORS_11X         0x010E  //o
#define OP_SET_DIO_IRQ_PARAMS_11X   0x0113  //o Configure DIO IRQ masks
#define OP_GET_IRQ_STATUS_11X       0x0115  //o Returns 4-byte IRQ status
#define OP_CLEAR_IRQ_11X            0x0116  //0114 0117g Write 4-byte mask to clear
#define OP_SET_TCXO_MODE_11X        0x0117  //oo Enable/configure TCXO
#define OP_SET_SLEEP_11X            0x011B  //o
#define OP_SET_STANDBY_11X          0x011C  //o Param: 0x00=RC, 0x01=XOSC
#define OP_SET_REGMODE_11X          0x0110  //o
#define OP_SET_FS_11X               0x011E  //o Enter frequency synthesis mode
#define OP_CALIBRATE_11X            0x010F  //oo Calibrate subsystems (bitmask)
#define OP_CALIBRATE_IMAGE_11X      0x0111  //o Image calibration for band
#define OP_SET_DIO_AS_RF_SWITCH_11X 0x0112  //o Map DIOs to TX/RX RF switch

// --- Radio group (0x02xx) ---
#define OP_RESET_STATS_11X          0x0200  //o
#define OP_GET_STATS_11X            0x0201  //o
#define OP_GET_PACKET_TYPE_11X      0x0202  //o
#define OP_GET_RX_BUFFER_STATUS_11X 0x0203  //o 
#define OP_GET_PACKET_STATUS_11X    0x0204  //o Returns RSSI, SNR, signal RSSI
#define OP_GET_RSSI_INST_11X        0x0205  //o Instantaneous RSSI
#define OP_SET_GFSK_SYNC_WORD_11X   0x0206  //o (not used in LoRa mode)
#define OP_SET_LORA_PUBLIC_NW_11X   0x0208  //0208  Toggle public/private sync word
#define OP_SET_RX_11X               0x0209  //0209 Start continuous/timeout RX
#define OP_SET_TX_11X               0x020A  //020A Start TX with timeout
#define OP_SET_RF_FREQ_11X          0x020B  //020B Set RF frequency (4 bytes PLL word)
#define OP_SET_PACKET_TYPE_11X      0x020E  //020E 0x00=GFSK, 0x01=LoRa
#define OP_SET_MODULATION_PARAMS_11X 0x020F //020F SF, BW, CR, LDRO
#define OP_SET_PACKET_PARAMS_11X    0x0210  //0210 Preamble, header, CRC, IQ
#define OP_SET_TX_PARAMS_11X        0x0211  //0211 Power level, ramp time
#define OP_SET_PA_CONFIG_11X        0x0215  //0215 PA select, supply, duty-cycle
#define OP_SET_RX_TX_FALLBACK_11X   0x0213  //0213 Post-TX/RX fallback mode
#define OP_SET_RX_DUTY_CYCLE_11X    0x0214  //0214 RX duty-cycle (Rx then sleep)
#define OP_SET_CAD_PARAMS_11X       0x020D  //020D CAD configuration



// ============================================================
// IRQ bit masks — 4 bytes (32-bit) on LR1121
// vs 2 bytes on SX126x
// ============================================================
#define IRQ_TX_DONE_MASK_11X              0x00000004UL
#define IRQ_RX_DONE_MASK_11X              0x00000008UL
#define IRQ_PREAMBLE_DET_MASK_11X         0x00000020UL
#define IRQ_HEADER_DET_MASK_11X           0x00000100UL
#define IRQ_HEADER_ERR_MASK_11X           0x00000200UL
#define IRQ_CRC_ERROR_MASK_11X            0x00000400UL
#define IRQ_ALL_MASK_11X                  0xFFFFFFFFUL

// ============================================================
// Packet / modulation mode bytes (same semantics as SX126x)
// ============================================================
#define MODE_LORA_11X               0x01
#define MODE_STDBY_RC_11X           0x00
#define MODE_STDBY_XOSC_11X         0x01
#define MODE_EXPLICIT_HEADER        0x00
#define MODE_IMPLICIT_HEADER        0x01

// ============================================================
// PA configuration constants (LR1121-specific)
// ============================================================
#define PA_SEL_LP_11X               0x00  // Low-power PA  (up to ~15 dBm)
#define PA_SEL_HP_11X               0x01  // High-power PA (up to ~22 dBm)
#define PA_SUPPLY_VREG_11X          0x00  // PA supplied from VREG
#define PA_SUPPLY_VBAT_11X          0x01  // PA supplied from VBAT

// ============================================================
// Register addresses (accessed via WriteRegMem32/ReadRegMem32)
// LR1121 uses 32-bit addresses; the 16-bit SX126x offsets are
// the same low 16 bits but the base differs.
// TODO: Verify these against LR1121 register map in datasheet.
// ============================================================
#define REG_SYNC_WORD_MSB_11X       0x00F30740UL
#define REG_SYNC_WORD_LSB_11X       0x00F30741UL  // NOTE: actual access is 32-bit aligned
#define REG_OCP_11X                 0x00F308E7UL
#define REG_RANDOM_GEN_11X          0x00F30819UL
// LR1121 does not have a direct LNA boost register like SX126x REG_LNA.
// Sensitivity optimisation is done through SetModulationParams LDRO flag.

// ============================================================
// Frequency calculation — identical to SX126x (32 MHz XTAL)
// ============================================================
#define XTAL_FREQ_11X (double)32000000
#define FREQ_DIV_11X  (double)pow(2.0, 25.0)
#define FREQ_STEP_11X (double)(XTAL_FREQ_11X / FREQ_DIV_11X)

// ============================================================
// TCXO voltage / trim codes for SetTcxoMode
// (Same voltage tiers as SX126x DIO3 TCXO control)
// ============================================================
#define TCXO_TRIM_1_6V_11X          0x00
#define TCXO_TRIM_1_7V_11X          0x01
#define TCXO_TRIM_1_8V_11X          0x02
#define TCXO_TRIM_2_2V_11X          0x03
#define TCXO_TRIM_2_4V_11X          0x04
#define TCXO_TRIM_2_7V_11X          0x05
#define TCXO_TRIM_3_0V_11X          0x06
#define TCXO_TRIM_3_3V_11X          0x07

// Sync word value used for Reticulum / private LoRa networks
#define SYNC_WORD_11X               0x1424

#define MAX_PKT_LENGTH 255

// ============================================================
// SPI selection — mirrors sx126x.cpp board handling
// ============================================================
#if BOARD_MODEL == BOARD_TECHO
  SPIClass spim3 = SPIClass(NRF_SPIM3, pin_miso, pin_sclk, pin_mosi);
  #define SPI spim3
#elif defined(NRF52840_XXAA)
  extern SPIClass spiModem;
  #define SPI spiModem
#endif

extern SPIClass SPI;

// ============================================================
// Constructor
// ============================================================
lr1121::lr1121() :
  _spiSettings(16E6, MSBFIRST, SPI_MODE0),
  _ss(LORA_DEFAULT_SS_PIN),
  _reset(LORA_DEFAULT_RESET_PIN),
  _dio0(LORA_DEFAULT_DIO0_PIN),
  _busy(LORA_DEFAULT_BUSY_PIN),
  _rxen(LORA_DEFAULT_RXEN_PIN),
  _frequency(0),
  _txp(0),
  _sf(0x07),
  _bw(0x04),
  _cr(0x01),
  _ldro(0x00),
  _packetIndex(0),
  _preambleLength(18),
  _implicitHeaderMode(0),
  _payloadLength(255),
  _crcMode(1),
  _fifo_tx_addr_ptr(0),
  _fifo_rx_addr_ptr(0),
  _packet({0}),
  _preinit_done(false),
  _dio0_risen(false),
  _onReceive(NULL)
{ setTimeout(0); }

// ============================================================
// preInit — SPI bus setup and chip detection
// ============================================================
bool lr1121::preInit() {
  pinMode(_ss, OUTPUT);
  digitalWrite(_ss, HIGH);

  #if BOARD_MODEL == BOARD_T3S3 || BOARD_MODEL == BOARD_HELTEC32_V3 || BOARD_MODEL == BOARD_HELTEC32_V4 || BOARD_MODEL == BOARD_TDECK || BOARD_MODEL == BOARD_XIAO_S3
    SPI.begin(pin_sclk, pin_miso, pin_mosi, pin_cs);
  #elif BOARD_MODEL == BOARD_TECHO
    SPI.setPins(pin_miso, pin_sclk, pin_mosi);
    SPI.begin();
  #else
    SPI.begin();
  #endif

  // On LR1121, GetVersion (0x0101) returns hardware/firmware version bytes.
  // A non-zero response on the version fields confirms the chip is alive.
  // TODO: Parse the version response properly; here we just check non-zero.
  long start = millis();
  uint8_t ver[4] = {0};
  while (((millis() - start) < 2000) && (millis() >= start)) {
    executeOpcodeRead(OP_GET_VERSION_11X, ver, 4);
    if (ver[0] != 0x00 || ver[1] != 0x00) { break; }
    delay(100);
  }
  if (ver[0] == 0x00 && ver[1] == 0x00) { return false; }

  _preinit_done = true;
  return true;
}

// ============================================================
// Low-level SPI helpers
//
// DIFFERENCE vs sx126x:
//   executeOpcode  — sends 2-byte command then N data bytes (write)
//   executeOpcodeRead — sends 2-byte command, 1 NOP, then reads N bytes
//   writeRegister  — wraps WriteRegMem32 (address must be 32-bit aligned)
//   readRegister   — wraps ReadRegMem32
// ============================================================

void lr1121::waitOnBusy() {
  unsigned long time = millis();
  if (_busy != -1) {
    while (digitalRead(_busy) == HIGH) {
      if (millis() >= (time + 100)) { break; }
    }
  }
}

void lr1121::executeOpcode(uint16_t opcode, uint8_t *buffer, uint8_t size) {
  waitOnBusy();
  digitalWrite(_ss, LOW);
  SPI.beginTransaction(_spiSettings);
  SPI.transfer((opcode >> 8) & 0xFF);  // high byte first
  SPI.transfer(opcode & 0xFF);         // low byte
  for (int i = 0; i < size; i++) { SPI.transfer(buffer[i]); }
  SPI.endTransaction();
  digitalWrite(_ss, HIGH);
}

// NOTE: Read transactions on LR1121 require one extra NOP byte after the
// command before data is valid — unlike SX126x which only needs it for
// register reads, not for opcode reads.
void lr1121::executeOpcodeRead(uint16_t opcode, uint8_t *buffer, uint8_t size) {
  waitOnBusy();
  digitalWrite(_ss, LOW);
  SPI.beginTransaction(_spiSettings);
  SPI.transfer((opcode >> 8) & 0xFF);
  SPI.transfer(opcode & 0xFF);
  SPI.transfer(0x00);  // mandatory NOP / status byte — DIFFERENT from SX126x
  for (int i = 0; i < size; i++) { buffer[i] = SPI.transfer(0x00); }
  SPI.endTransaction();
  digitalWrite(_ss, HIGH);
}

// WriteRegMem32: cmd | 32-bit address | 32-bit value (only low byte used for
// byte-wide logical registers; the hardware always transacts in 32-bit words).
// NOTE: Address and value are sent as big-endian 32-bit words.
void lr1121::writeRegister(uint32_t address, uint8_t value) {
  waitOnBusy();
  // number of 32-bit words = 1
  uint8_t count = 1;
  uint32_t aligned_addr = address & ~0x3UL;
  uint8_t  byte_offset  = address & 0x3UL;

  // Read-modify-write: read existing 32-bit word, poke our byte, write back.
  uint8_t word[4] = {0};
  waitOnBusy();
  digitalWrite(_ss, LOW);
  SPI.beginTransaction(_spiSettings);
  SPI.transfer((OP_READ_REGMEM32_11X >> 8) & 0xFF);
  SPI.transfer(OP_READ_REGMEM32_11X & 0xFF);
  // Address (4 bytes) + count byte
  SPI.transfer((aligned_addr >> 24) & 0xFF);
  SPI.transfer((aligned_addr >> 16) & 0xFF);
  SPI.transfer((aligned_addr >>  8) & 0xFF);
  SPI.transfer((aligned_addr      ) & 0xFF);
  SPI.transfer(count);
  SPI.transfer(0x00); // NOP for read
  for (int i = 0; i < 4; i++) { word[i] = SPI.transfer(0x00); }
  SPI.endTransaction();
  digitalWrite(_ss, HIGH);

  word[byte_offset] = value;

  waitOnBusy();
  digitalWrite(_ss, LOW);
  SPI.beginTransaction(_spiSettings);
  SPI.transfer((OP_WRITE_REGMEM32_11X >> 8) & 0xFF);
  SPI.transfer(OP_WRITE_REGMEM32_11X & 0xFF);
  SPI.transfer((aligned_addr >> 24) & 0xFF);
  SPI.transfer((aligned_addr >> 16) & 0xFF);
  SPI.transfer((aligned_addr >>  8) & 0xFF);
  SPI.transfer((aligned_addr      ) & 0xFF);
  for (int i = 0; i < 4; i++) { SPI.transfer(word[i]); }
  SPI.endTransaction();
  digitalWrite(_ss, HIGH);
}

uint8_t lr1121::readRegister(uint32_t address) {
  uint32_t aligned_addr = address & ~0x3UL;
  uint8_t  byte_offset  = address & 0x3UL;
  uint8_t  count = 1;
  uint8_t  word[4] = {0};

  waitOnBusy();
  digitalWrite(_ss, LOW);
  SPI.beginTransaction(_spiSettings);
  SPI.transfer((OP_READ_REGMEM32_11X >> 8) & 0xFF);
  SPI.transfer(OP_READ_REGMEM32_11X & 0xFF);
  SPI.transfer((aligned_addr >> 24) & 0xFF);
  SPI.transfer((aligned_addr >> 16) & 0xFF);
  SPI.transfer((aligned_addr >>  8) & 0xFF);
  SPI.transfer((aligned_addr      ) & 0xFF);
  SPI.transfer(count);
  SPI.transfer(0x00); // NOP
  for (int i = 0; i < 4; i++) { word[i] = SPI.transfer(0x00); }
  SPI.endTransaction();
  digitalWrite(_ss, HIGH);

  return word[byte_offset];
}

// ============================================================
// Buffer I/O
// WriteBuffer8/ReadBuffer8 use 8-bit byte-addressing into the
// TX/RX FIFO, identical in concept to SX126x FIFO ops.
// ============================================================
void lr1121::writeBuffer(const uint8_t* buffer, size_t size) {
  waitOnBusy();
  digitalWrite(_ss, LOW);
  SPI.beginTransaction(_spiSettings);
  SPI.transfer((OP_WRITE_BUFFER8_11X >> 8) & 0xFF);
  SPI.transfer(OP_WRITE_BUFFER8_11X & 0xFF);
  SPI.transfer(_fifo_tx_addr_ptr);
  for (int i = 0; i < (int)size; i++) { SPI.transfer(buffer[i]); _fifo_tx_addr_ptr++; }
  SPI.endTransaction();
  digitalWrite(_ss, HIGH);
}

void lr1121::readBuffer(uint8_t* buffer, size_t size) {
  waitOnBusy();
  digitalWrite(_ss, LOW);
  SPI.beginTransaction(_spiSettings);
  SPI.transfer((OP_READ_BUFFER8_11X >> 8) & 0xFF);
  SPI.transfer(OP_READ_BUFFER8_11X & 0xFF);
  SPI.transfer(_fifo_rx_addr_ptr);
  SPI.transfer(0x00); // NOP / offset byte before data
  for (int i = 0; i < (int)size; i++) { buffer[i] = SPI.transfer(0x00); }
  SPI.endTransaction();
  digitalWrite(_ss, HIGH);
}

// ============================================================
// Modem parameter setters
// setModulationParams / setPacketParams — same logical fields as
// SX126x but sent with a different opcode.
// ============================================================
void lr1121::setModulationParams(uint8_t sf, uint8_t bw, uint8_t cr, int ldro) {
  uint8_t buf[4];
  buf[0] = sf;
  buf[1] = bw;
  buf[2] = cr;
  buf[3] = (uint8_t)ldro;
  executeOpcode(OP_SET_MODULATION_PARAMS_11X, buf, 4);
}

void lr1121::setPacketParams(long preamble_symbols, uint8_t headermode, uint8_t payload_length, uint8_t crc) {
  uint8_t buf[6];
  buf[0] = (uint8_t)((preamble_symbols & 0xFF00) >> 8);
  buf[1] = (uint8_t)( preamble_symbols & 0x00FF);
  buf[2] = headermode;
  buf[3] = payload_length;
  buf[4] = crc;
  buf[5] = 0x00; // standard IQ (no inversion)
  executeOpcode(OP_SET_PACKET_PARAMS_11X, buf, 6);
}

// ============================================================
// Reset
// ============================================================
void lr1121::reset(void) {
  if (_reset != -1) {
    pinMode(_reset, OUTPUT);
    digitalWrite(_reset, LOW);
    delay(10);
    digitalWrite(_reset, HIGH);
    delay(10);
  }
}

// ============================================================
// Calibration
// DIFFERENCE vs sx126x:
//   - SetRegMode must be called first to configure the power supply
//     regulator (LDO or DC-DC) before calibration.
//   - The calibrate bitmask fields are similar but the LR1121 adds
//     more calibration targets (e.g., ADC).
// ============================================================
void lr1121::calibrate(void) {
  uint8_t mode_byte = MODE_STDBY_RC_11X;
  executeOpcode(OP_SET_STANDBY_11X, &mode_byte, 1);

  // Configure regulator: 0x01 = DC-DC, 0x00 = LDO.
  // Most boards with LR1121 use DC-DC; adjust per board as needed.
  #if defined(LR1121_USE_DCDC)
    uint8_t regmode = 0x01;
  #else
    uint8_t regmode = 0x00;
  #endif
  executeOpcode(OP_SET_REGMODE_11X, &regmode, 1);

  // Calibrate all subsystems: RC64k, RC13M, PLL, ADC, image.
  // 0x7F = all bits set for calibration targets.
  uint8_t cal = 0x7F;
  executeOpcode(OP_CALIBRATE_11X, &cal, 1);

  delay(5);
  waitOnBusy();
}

void lr1121::calibrate_image(long frequency) {
  // Image calibration band pairs — same frequency ranges as SX126x.
  uint8_t image_freq[2] = {0};
  if      (frequency >= 430E6 && frequency <= 440E6) { image_freq[0] = 0x6B; image_freq[1] = 0x6F; }
  else if (frequency >= 470E6 && frequency <= 510E6) { image_freq[0] = 0x75; image_freq[1] = 0x81; }
  else if (frequency >= 779E6 && frequency <= 787E6) { image_freq[0] = 0xC1; image_freq[1] = 0xC5; }
  else if (frequency >= 863E6 && frequency <= 870E6) { image_freq[0] = 0xD7; image_freq[1] = 0xDB; }
  else if (frequency >= 902E6 && frequency <= 928E6) { image_freq[0] = 0xE1; image_freq[1] = 0xE9; }
  executeOpcode(OP_CALIBRATE_IMAGE_11X, image_freq, 2);
  waitOnBusy();
}

// ============================================================
// loraMode — set packet type to LoRa
// ============================================================
void lr1121::loraMode() {
  uint8_t mode = MODE_LORA_11X;
  executeOpcode(OP_SET_PACKET_TYPE_11X, &mode, 1);
}

// ============================================================
// rxAntEnable
// ============================================================
void lr1121::rxAntEnable() {
  if (_rxen != -1) { digitalWrite(_rxen, HIGH); }
}

// ============================================================
// begin — full modem initialisation
// ============================================================
int lr1121::begin(long frequency) {
  reset();

  if (_busy != -1) { pinMode(_busy, INPUT); }
  if (!_preinit_done) { if (!preInit()) { return false; } }
  if (_rxen != -1) { pinMode(_rxen, OUTPUT); }

  calibrate();
  calibrate_image(frequency);
  enableTCXO();
  loraMode();
  standby();

  setSyncWord(SYNC_WORD_11X);

  // Configure DIOs as RF switch if the board uses them.
  // LR1121 has a dedicated SetDioAsRfSwitch command unlike SX126x DIO2.
  #if defined(LR1121_USE_DIO_RF_SWITCH)
    // TODO: Set DIO mapping for RF switch based on board schematic.
    // Params: enable(1), standby_mode, rx_mode, tx_mode, tx_hp_mode, tx_hf_mode, gnss_mode, wifi_mode
    uint8_t rf_sw[8] = {0x01, 0x00, 0x02, 0x01, 0x04, 0x00, 0x00, 0x00};
    executeOpcode(OP_SET_DIO_AS_RF_SWITCH_11X, rf_sw, 8);
  #endif

  rxAntEnable();
  setFrequency(frequency);
  setTxPower(2);
  enableCrc();

  // LR1121 does not have a separate LNA boost register like SX126x REG_LNA.
  // Sensitivity is maximised by ensuring LDRO is set correctly in
  // setModulationParams, which handleLowDataRate() manages.

  // Set TX/RX buffer base addresses to 0.
  uint8_t basebuf[2] = {0, 0};
  executeOpcode(OP_SET_BUFFER_BASE_ADDR_11X, basebuf, 2);

  setModulationParams(_sf, _bw, _cr, _ldro);
  setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);

  return 1;
}

void lr1121::end() { sleep(); SPI.end(); _preinit_done = false; }

// ============================================================
// beginPacket / endPacket
// ============================================================
int lr1121::beginPacket(int implicitHeader) {
  standby();
  if (implicitHeader) { implicitHeaderMode(); }
  else                { explicitHeaderMode(); }

  _payloadLength = 0;
  _fifo_tx_addr_ptr = 0;
  setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);
  return 1;
}

int lr1121::endPacket() {
  setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);

  // SetTx with timeout = 0 → single shot TX.
  // LR1121 timeout is a 24-bit value (3 bytes), same as SX126x.
  uint8_t timeout[3] = {0};
  executeOpcode(OP_SET_TX_11X, timeout, 3);

  // Poll IRQ status until TX_DONE or timeout.
  // NOTE: IRQ status is 4 bytes on LR1121, not 2.
  uint8_t buf[4] = {0};
  bool timed_out = false;
  uint32_t w_timeout = millis() + LORA_MODEM_TIMEOUT_MS;
  while ((millis() < w_timeout) && ((buf[0] & (IRQ_TX_DONE_MASK_11X >> 24)) == 0)) {
    memset(buf, 0, sizeof(buf));
    executeOpcodeRead(OP_GET_IRQ_STATUS_11X, buf, 4);
    yield();
  }

  if (!(millis() < w_timeout)) { timed_out = true; }

  // Clear IRQs — write 4-byte all-ones mask.
  uint8_t clear[4] = {0xFF, 0xFF, 0xFF, 0xFF};
  executeOpcode(OP_CLEAR_IRQ_11X, clear, 4);

  if (timed_out) { return 0; } else { return 1; }
}

// ============================================================
// dcd — carrier / preamble detection
// ============================================================
unsigned long preamble_detected_at_11x = 0;
extern long lora_preamble_time_ms;
extern long lora_header_time_ms;
bool false_preamble_detected_11x = false;

bool lr1121::dcd() {
  uint8_t buf[4] = {0};
  executeOpcodeRead(OP_GET_IRQ_STATUS_11X, buf, 4);
  // Reconstruct 32-bit IRQ word (big-endian).
  uint32_t irq = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
                 ((uint32_t)buf[2] <<  8) |  (uint32_t)buf[3];
  uint32_t now = millis();

  bool header_detected  = (irq & IRQ_HEADER_DET_MASK_11X)   != 0;
  bool preamble_detected = (irq & IRQ_PREAMBLE_DET_MASK_11X) != 0;
  bool carrier_detected = header_detected || preamble_detected;

  if (preamble_detected) {
    if (preamble_detected_at_11x == 0) { preamble_detected_at_11x = now; }
    if (now - preamble_detected_at_11x > lora_preamble_time_ms + lora_header_time_ms) {
      preamble_detected_at_11x = 0;
      if (!header_detected) { false_preamble_detected_11x = true; }
      uint8_t clear[4];
      clear[0] = (uint8_t)(IRQ_PREAMBLE_DET_MASK_11X >> 24);
      clear[1] = (uint8_t)(IRQ_PREAMBLE_DET_MASK_11X >> 16);
      clear[2] = (uint8_t)(IRQ_PREAMBLE_DET_MASK_11X >>  8);
      clear[3] = (uint8_t)(IRQ_PREAMBLE_DET_MASK_11X      );
      executeOpcode(OP_CLEAR_IRQ_11X, clear, 4);
    }
  }

  if (false_preamble_detected_11x) { lr1121_modem.receive(); false_preamble_detected_11x = false; }
  return carrier_detected;
}

// ============================================================
// RSSI / SNR
// GetPacketStatus on LR1121 returns 3 bytes: rssi_pkt, snr_pkt, signal_rssi_pkt.
// Same byte order as SX126x, same formula.
// ============================================================
uint8_t lr1121::currentRssiRaw() {
  uint8_t byte = 0;
  executeOpcodeRead(OP_GET_RSSI_INST_11X, &byte, 1);
  return byte;
}

int ISR_VECT lr1121::currentRssi() {
  uint8_t byte = 0;
  executeOpcodeRead(OP_GET_RSSI_INST_11X, &byte, 1);
  int rssi = -(int(byte)) / 2;
  #if HAS_LORA_LNA
    rssi -= LORA_LNA_GAIN;
  #endif
  return rssi;
}

uint8_t lr1121::packetRssiRaw() {
  uint8_t buf[3] = {0};
  executeOpcodeRead(OP_GET_PACKET_STATUS_11X, buf, 3);
  return buf[0]; // rssi_pkt
}

int ISR_VECT lr1121::packetRssi() {
  uint8_t buf[3] = {0};
  executeOpcodeRead(OP_GET_PACKET_STATUS_11X, buf, 3);
  int pkt_rssi = -(int)buf[0] / 2;
  #if HAS_LORA_LNA
    pkt_rssi -= LORA_LNA_GAIN;
  #endif
  return pkt_rssi;
}

int ISR_VECT lr1121::packetRssi(uint8_t pkt_snr_raw) {
  uint8_t buf[3] = {0};
  executeOpcodeRead(OP_GET_PACKET_STATUS_11X, buf, 3);
  return -(int)buf[0] / 2;
}

uint8_t ISR_VECT lr1121::packetSnrRaw() {
  uint8_t buf[3] = {0};
  executeOpcodeRead(OP_GET_PACKET_STATUS_11X, buf, 3);
  return buf[1]; // snr_pkt
}

float ISR_VECT lr1121::packetSnr() {
  uint8_t buf[3] = {0};
  executeOpcodeRead(OP_GET_PACKET_STATUS_11X, buf, 3);
  return float(buf[1]) * 0.25;
}

long lr1121::packetFrequencyError() {
  // TODO: LR1121 may expose frequency error — check GetPacketStatus fields.
  return 0;
}

// ============================================================
// Stream interface
// ============================================================
size_t lr1121::write(uint8_t byte) { return write(&byte, sizeof(byte)); }
size_t lr1121::write(const uint8_t *buffer, size_t size) {
  if ((_payloadLength + size) > MAX_PKT_LENGTH) { size = MAX_PKT_LENGTH - _payloadLength; }
  writeBuffer(buffer, size);
  _payloadLength = _payloadLength + size;
  return size;
}

int ISR_VECT lr1121::available() {
  uint8_t buf[2] = {0};
  executeOpcodeRead(OP_GET_RX_BUFFER_STATUS_11X, buf, 2);
  return buf[0] - _packetIndex;
}

int ISR_VECT lr1121::read() {
  if (!available()) { return -1; }
  if (_packetIndex == 0) {
    uint8_t rxbuf[2] = {0};
    executeOpcodeRead(OP_GET_RX_BUFFER_STATUS_11X, rxbuf, 2);
    int size = rxbuf[0];
    _fifo_rx_addr_ptr = rxbuf[1];
    readBuffer(_packet, size);
  }
  uint8_t byte = _packet[_packetIndex];
  _packetIndex++;
  return byte;
}

int lr1121::peek() {
  if (!available()) { return -1; }
  if (_packetIndex == 0) {
    uint8_t rxbuf[2] = {0};
    executeOpcodeRead(OP_GET_RX_BUFFER_STATUS_11X, rxbuf, 2);
    int size = rxbuf[0];
    _fifo_rx_addr_ptr = rxbuf[1];
    readBuffer(_packet, size);
  }
  return _packet[_packetIndex];
}

void lr1121::flush() { }

// ============================================================
// onReceive / DIO0 interrupt
// ============================================================
void lr1121::onReceive(void(*callback)(int)) {
  _onReceive = callback;

  if (callback) {
    pinMode(_dio0, INPUT);

    // SetDioIrqParams: 4-byte global IRQ mask, 4-byte DIO1 mask,
    // 4-byte DIO2 mask. Map RX_DONE to DIO1 (wired as DIO0 in this code).
    // TODO: Verify DIO mapping against board schematic.
    uint8_t buf[12];
    // Global mask — enable all
    buf[0]  = 0xFF; buf[1]  = 0xFF; buf[2]  = 0xFF; buf[3]  = 0xFF;
    // DIO1 mask — RX_DONE
    buf[4]  = (uint8_t)(IRQ_RX_DONE_MASK_11X >> 24);
    buf[5]  = (uint8_t)(IRQ_RX_DONE_MASK_11X >> 16);
    buf[6]  = (uint8_t)(IRQ_RX_DONE_MASK_11X >>  8);
    buf[7]  = (uint8_t)(IRQ_RX_DONE_MASK_11X      );
    // DIO2 mask — nothing
    buf[8]  = 0x00; buf[9]  = 0x00; buf[10] = 0x00; buf[11] = 0x00;
    executeOpcode(OP_SET_DIO_IRQ_PARAMS_11X, buf, 12);

    #ifdef SPI_HAS_NOTUSINGINTERRUPT
      SPI.usingInterrupt(digitalPinToInterrupt(_dio0));
    #endif
    attachInterrupt(digitalPinToInterrupt(_dio0), lr1121::onDio0Rise, RISING);

  } else {
    detachInterrupt(digitalPinToInterrupt(_dio0));
    #ifdef SPI_HAS_NOTUSINGINTERRUPT
      SPI.notUsingInterrupt(digitalPinToInterrupt(_dio0));
    #endif
  }
}

// ============================================================
// receive — start continuous RX
// ============================================================
void lr1121::receive(int size) {
  if (size > 0) {
    implicitHeaderMode();
    _payloadLength = size;
    setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);
  } else {
    explicitHeaderMode();
  }

  if (_rxen != -1) { rxAntEnable(); }
  uint8_t mode[3] = {0xFF, 0xFF, 0xFF}; // Continuous mode (timeout = 0xFFFFFF)
  executeOpcode(OP_SET_RX_11X, mode, 3);
}

// ============================================================
// standby / sleep
// ============================================================
void lr1121::standby() {
  uint8_t byte = MODE_STDBY_XOSC_11X;
  executeOpcode(OP_SET_STANDBY_11X, &byte, 1);
}

void lr1121::sleep() {
  uint8_t byte = 0x00;
  executeOpcode(OP_SET_SLEEP_11X, &byte, 1);
}

// ============================================================
// enableTCXO
// DIFFERENCE vs sx126x:
//   SX126x uses OP_DIO3_TCXO_CTRL_6X (0x97) which configures DIO3
//   as the TCXO power supply pin.
//   LR1121 uses SetTcxoMode (0x0118) with voltage trim and a 24-bit
//   timeout, which is structurally the same but with a 16-bit opcode.
// ============================================================
void lr1121::enableTCXO() {
  #if HAS_TCXO
    #if BOARD_MODEL == BOARD_RAK4631 || BOARD_MODEL == BOARD_HELTEC32_V3 || BOARD_MODEL == BOARD_XIAO_S3
      uint8_t buf[4] = {TCXO_TRIM_3_3V_11X, 0x00, 0x00, 0xFF};
    #elif BOARD_MODEL == BOARD_TBEAM
      uint8_t buf[4] = {TCXO_TRIM_1_8V_11X, 0x00, 0x00, 0xFF};
    #elif BOARD_MODEL == BOARD_TDECK
      uint8_t buf[4] = {TCXO_TRIM_1_8V_11X, 0x00, 0x00, 0xFF};
    #elif BOARD_MODEL == BOARD_TBEAM_S_V1
      uint8_t buf[4] = {TCXO_TRIM_1_8V_11X, 0x00, 0x00, 0xFF};
    #elif BOARD_MODEL == BOARD_T3S3
      uint8_t buf[4] = {TCXO_TRIM_1_8V_11X, 0x00, 0x00, 0xFF};
    #elif BOARD_MODEL == BOARD_HELTEC_T114
      uint8_t buf[4] = {TCXO_TRIM_1_8V_11X, 0x00, 0x00, 0xFF};
    #elif BOARD_MODEL == BOARD_TECHO
      uint8_t buf[4] = {TCXO_TRIM_1_8V_11X, 0x00, 0x00, 0xFF};
    #elif BOARD_MODEL == BOARD_HELTEC32_V4
      uint8_t buf[4] = {TCXO_TRIM_1_8V_11X, 0x00, 0x00, 0xFF};
    #else
      uint8_t buf[4] = {TCXO_TRIM_1_8V_11X, 0x00, 0x00, 0xFF};
    #endif
    executeOpcode(OP_SET_TCXO_MODE_11X, buf, 4);
  #endif
}

void lr1121::disableTCXO() { }

// ============================================================
// setTxPower
// DIFFERENCE vs sx126x:
//   SX126x SetPaConfig has 4 fields: PADutyCycle, HPMax, DeviceSel, PALut.
//   LR1121 SetPaConfig has 4 fields: pa_sel, pa_reg_supply, pa_duty_cycle,
//   pa_hp_sel. pa_sel chooses LP (0) or HP (1) PA.
//   Power range: HP PA = -9 to +22 dBm, LP PA = -17 to +15 dBm.
// ============================================================
void lr1121::setTxPower(int level, int outputPin) {
  uint8_t pa_buf[4];

  if (level > 15) {
    // High-power PA path
    pa_buf[0] = PA_SEL_HP_11X;
    pa_buf[1] = PA_SUPPLY_VBAT_11X;  // HP PA is fed from VBAT
    pa_buf[2] = 0x04;                 // duty cycle — adjust for efficiency
    pa_buf[3] = 0x07;                 // pa_hp_sel — max for 22 dBm
    if (level > 22) { level = 22; }
  } else {
    // Low-power PA path
    pa_buf[0] = PA_SEL_LP_11X;
    pa_buf[1] = PA_SUPPLY_VREG_11X;  // LP PA is fed from VREG
    pa_buf[2] = 0x04;
    pa_buf[3] = 0x00;                 // pa_hp_sel unused for LP PA
    if (level > 15) { level = 15; }
  }

  if (level < -9) { level = -9; }
  executeOpcode(OP_SET_PA_CONFIG_11X, pa_buf, 4);

  // SetTxParams: power (signed byte), ramp time.
  // Ramp time 0x02 = 40 µs (same as SX126x).
  uint8_t tx_buf[2];
  tx_buf[0] = (uint8_t)level;
  tx_buf[1] = 0x02;
  executeOpcode(OP_SET_TX_PARAMS_11X, tx_buf, 2);

  _txp = level;
}

uint8_t lr1121::getTxPower() { return _txp; }

// ============================================================
// Frequency
// ============================================================
void lr1121::setFrequency(long frequency) {
  _frequency = frequency;
  uint8_t buf[4];
  uint32_t freq = (uint32_t)((double)frequency / (double)FREQ_STEP_11X);
  buf[0] = ((freq >> 24) & 0xFF);
  buf[1] = ((freq >> 16) & 0xFF);
  buf[2] = ((freq >>  8) & 0xFF);
  buf[3] = ( freq        & 0xFF);
  executeOpcode(OP_SET_RF_FREQ_11X, buf, 4);
}

uint32_t lr1121::getFrequency() { return _frequency; }

// ============================================================
// Spreading factor / bandwidth / coding rate
// ============================================================
void lr1121::setSpreadingFactor(int sf) {
  if (sf < 5)       { sf = 5; }
  else if (sf > 12) { sf = 12; }
  _sf = sf;
  handleLowDataRate();
  setModulationParams(sf, _bw, _cr, _ldro);
}

long lr1121::getSignalBandwidth() {
  // Bandwidth register codes are identical to SX126x.
  switch (_bw) {
    case 0x00: return 7.8E3;
    case 0x01: return 15.6E3;
    case 0x02: return 31.25E3;
    case 0x03: return 62.5E3;
    case 0x04: return 125E3;
    case 0x05: return 250E3;
    case 0x06: return 500E3;
    case 0x08: return 10.4E3;
    case 0x09: return 20.8E3;
    case 0x0A: return 41.7E3;
  }
  return 0;
}

extern bool lora_low_datarate;
void lr1121::handleLowDataRate() {
  if ( long( (1<<_sf) / (getSignalBandwidth()/1000)) > 16)
    { _ldro = 0x01; lora_low_datarate = true;  }
  else
    { _ldro = 0x00; lora_low_datarate = false; }
}

void lr1121::optimizeModemSensitivity() { }

void lr1121::setSignalBandwidth(long sbw) {
  if      (sbw <= 7.8E3)    { _bw = 0x00; }
  else if (sbw <= 10.4E3)   { _bw = 0x08; }
  else if (sbw <= 15.6E3)   { _bw = 0x01; }
  else if (sbw <= 20.8E3)   { _bw = 0x09; }
  else if (sbw <= 31.25E3)  { _bw = 0x02; }
  else if (sbw <= 41.7E3)   { _bw = 0x0A; }
  else if (sbw <= 62.5E3)   { _bw = 0x03; }
  else if (sbw <= 125E3)    { _bw = 0x04; }
  else if (sbw <= 250E3)    { _bw = 0x05; }
  else                      { _bw = 0x06; }
  handleLowDataRate();
  setModulationParams(_sf, _bw, _cr, _ldro);
  optimizeModemSensitivity();
}

void lr1121::setCodingRate4(int denominator) {
  if (denominator < 5) { denominator = 5; }
  else if (denominator > 8) { denominator = 8; }
  _cr = denominator - 4;
  setModulationParams(_sf, _bw, _cr, _ldro);
}

void lr1121::setPreambleLength(long preamble_symbols) {
  _preambleLength = preamble_symbols;
  setPacketParams(preamble_symbols, _implicitHeaderMode, _payloadLength, _crcMode);
}

// ============================================================
// setSyncWord
// DIFFERENCE vs sx126x:
//   SX126x writes to REG_SYNC_WORD_MSB/LSB_6X via 16-bit register addresses.
//   LR1121 uses WriteRegMem32 with 32-bit addresses.
//   NOTE: The REG_SYNC_WORD addresses below need verifying against the
//   LR1121 register map. The SX126x values (0x0740/0x0741) are used as a
//   starting point — they may differ for the LR1121.
// ============================================================
void lr1121::setSyncWord(uint16_t sw) {
  writeRegister(REG_SYNC_WORD_MSB_11X, 0x14);
  writeRegister(REG_SYNC_WORD_LSB_11X, 0x24);
}

// ============================================================
// Pin configuration
// ============================================================
void lr1121::setPins(int ss, int reset, int dio0, int busy, int rxen) {
  _ss    = ss;
  _reset = reset;
  _dio0  = dio0;
  _busy  = busy;
  _rxen  = rxen;
}

void lr1121::setSPIFrequency(uint32_t frequency) {
  _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0);
}

// ============================================================
// CRC / header mode helpers
// ============================================================
void lr1121::enableCrc()         { _crcMode = 1; setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode); }
void lr1121::disableCrc()        { _crcMode = 0; setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode); }
void lr1121::explicitHeaderMode(){ _implicitHeaderMode = 0; setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode); }
void lr1121::implicitHeaderMode(){ _implicitHeaderMode = 1; setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode); }

// ============================================================
// random — use random number generator register
// ============================================================
byte lr1121::random() { return readRegister(REG_RANDOM_GEN_11X); }

// ============================================================
// dumpRegisters — diagnostic dump (limited: reads only known addresses)
// ============================================================
void lr1121::dumpRegisters(Stream& out) {
  // LR1121 does not have a contiguous readable register space like SX127x.
  // Print the known sync-word registers as a sanity check.
  out.print("SyncWord MSB: 0x");
  out.println(readRegister(REG_SYNC_WORD_MSB_11X), HEX);
  out.print("SyncWord LSB: 0x");
  out.println(readRegister(REG_SYNC_WORD_LSB_11X), HEX);
}

// ============================================================
// DIO0 ISR — deferred to pollDio0() (same pattern as sx126x)
// ============================================================
void ISR_VECT lr1121::handleDio0Rise() { _dio0_risen = true; }

void lr1121::pollDio0() {
  if (!_dio0_risen) return;
  _dio0_risen = false;

  uint8_t buf[4] = {0};
  executeOpcodeRead(OP_GET_IRQ_STATUS_11X, buf, 4);

  uint8_t clear[4] = {0xFF, 0xFF, 0xFF, 0xFF};
  executeOpcode(OP_CLEAR_IRQ_11X, clear, 4);

  uint32_t irq = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
                 ((uint32_t)buf[2] <<  8) |  (uint32_t)buf[3];

  if ((irq & IRQ_CRC_ERROR_MASK_11X) == 0) {
    _packetIndex = 0;
    uint8_t rxbuf[2] = {0};
    executeOpcodeRead(OP_GET_RX_BUFFER_STATUS_11X, rxbuf, 2);
    int packetLength = rxbuf[0];
    if (_onReceive) { _onReceive(packetLength); }
  }
}

void ISR_VECT lr1121::onDio0Rise() { lr1121_modem.handleDio0Rise(); }

lr1121 lr1121_modem;

#endif
