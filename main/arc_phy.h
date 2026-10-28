#pragma once

#include <stdbool.h>
#include <stdint.h>

#define ARC_RX_STAGE_COUNT 9u
#define ARC_VENDOR_GAIN_MAX 89u

/* The installed RX gain table, reconstructed exactly as the pinned vendor
 * generator builds it (phy_gen_rx_gain_table, esp-phy-lib 59c1234): entry
 * k of stage s has counter c = start[s] + k, BB = {1,3,...,127}[c / 6] and
 * fine = 5 - c % 6. The 5 GHz band (phy_param[0x2a] != 0) uses its own
 * constant spans / start counters / RF codes and its maximum at +0x126. */
typedef struct {
    uint8_t spans[ARC_RX_STAGE_COUNT];
    uint8_t start[ARC_RX_STAGE_COUNT];
    uint16_t rf_code[ARC_RX_STAGE_COUNT];
    uint8_t max_index;
    bool band5;
    bool runtime_spans_valid;
} arc_gain_table_t;

#define ARC_5G_GAIN_MAX 83u     /* 84 entries, counters 12..41 in the last stage */

typedef struct {
    uint8_t gain_index;
    uint8_t rf_stage;
    uint16_t rf_code;
    uint8_t bb_code;
    uint8_t fine_code;
    uint32_t packed_state;
} arc_gain_tuple_t;

/* Capture the gain-table description produced by the pinned vendor PHY.
 * This function is read-only and must run after normal PHY initialization. */
void arc_phy_capture_gain_table(arc_gain_table_t *table);

/* Pure helpers are kept public so the closed-PHY reconstruction can be
 * regression-tested on the host. */
/* spans == NULL: the 5 GHz table (this receiver's band); otherwise the
 * 2.4 GHz table with those runtime spans (+0x422). */
void arc_gain_table_from_bytes(arc_gain_table_t *table,
                               const uint8_t spans[ARC_RX_STAGE_COUNT],
                               uint8_t max_index);
bool arc_gain_tuple_decode(const arc_gain_table_t *table, uint8_t gain_index,
                           arc_gain_tuple_t *tuple);
uint8_t arc_gain_highest_rf_stage_start(const arc_gain_table_t *table);

/* The installed table as the PHY's AGC memory holds it (modem memory window
 * at 0x600ae000, as vendor apb_get_mem() in librftest.a maps it; board
 * 2026-10-21: all 84 rows equal the reconstruction above). One entry is four
 * words: RF code = w2 << 5 | w1 >> 27, fine = (w1 >> 24) & 7,
 * BB = (w1 >> 17) & 0x7f. The other bits are constant per RF stage and not
 * decoded. */
#define ARC_AGC_ENTRY_WORDS 4u
void arc_gain_entry_from_agc(const uint32_t w[ARC_AGC_ENTRY_WORDS], uint16_t *rf_code,
                             uint8_t *bb_code, uint8_t *fine_code);
/* The row's PBUS DC words in the same entry, as vendor phy_wr_rx_gain_mem
 * packs them (Ghidra, pinned libphy): bank 1 (per RF stage and frequency
 * group) I = w1[16:8], Q = w0[30:22]; bank 2 I = w1[7:0] << 1 | w0[31],
 * Q = w0[21:13]. */
void arc_gain_entry_dc(const uint32_t w[ARC_AGC_ENTRY_WORDS], int bank1[2], int bank2[2]);
/* Rows 0..max_index whose tuple equals the AGC entry; *first_bad is the first
 * row that differs (or is missing from words), -1 when none. */
unsigned arc_gain_table_check_agc(const arc_gain_table_t *table, const uint32_t *words,
                                  unsigned n_words, int *first_bad);
