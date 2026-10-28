/* C5VRX-5 by Twotoz and the C5VRX contributors: host test of the AGC-memory
 * check of the 5 GHz gain table (main/arc_phy.c arc_gain_table_check_agc).
 * The words are the PHY's AGC memory read on a XIAO ESP32-C5 (rev 1.0,
 * esp-phy-lib of ESP-IDF 6.0.2) on 2026-10-21 through the modem memory
 * window, rows G0..G83.
 *
 *   cc -O2 -std=c11 -I../main test_agc_table.c ../main/arc_phy.c -o t && ./t
 */
#include <stdio.h>
#include <string.h>
#include "arc_phy.h"

unsigned char phy_param[2048];

static const uint32_t s_board[(ARC_5G_GAIN_MAX + 1u) * ARC_AGC_ENTRY_WORDS] = {
    0x41defb7a, 0xc50f047d, 0x00000000, 0x00000000,   /* G0 */
    0x41defb7a, 0xc40f047d, 0x00000000, 0x00000000,   /* G1 */
    0x41defb7a, 0xc30f047d, 0x00000000, 0x00000000,   /* G2 */
    0x41defb7a, 0xc20f047d, 0x00000000, 0x00000000,   /* G3 */
    0x41defb7a, 0xc10f047d, 0x00000000, 0x00000000,   /* G4 */
    0x41defb7a, 0xc00f047d, 0x00000000, 0x00000000,   /* G5 */
    0xc1df5b7a, 0xc51f047e, 0x00000000, 0x00000000,   /* G6 */
    0xc1df5b7a, 0xc41f047e, 0x00000000, 0x00000000,   /* G7 */
    0xc1df5b7a, 0xc31f047e, 0x00000000, 0x00000000,   /* G8 */
    0xc1df5b7a, 0xc21f047e, 0x00000000, 0x00000000,   /* G9 */
    0x425efb7a, 0xa40f067d, 0x00000000, 0x00000000,   /* G10 */
    0x425efb7a, 0xa30f067d, 0x00000000, 0x00000000,   /* G11 */
    0x425efb7a, 0xa20f067d, 0x00000000, 0x00000000,   /* G12 */
    0x425efb7a, 0xa10f067d, 0x00000000, 0x00000000,   /* G13 */
    0x425efb7a, 0xa00f067d, 0x00000000, 0x00000000,   /* G14 */
    0xc25f5b7a, 0xa51f067e, 0x00000000, 0x00000000,   /* G15 */
    0xc25f5b7a, 0xa41f067e, 0x00000000, 0x00000000,   /* G16 */
    0x431efb7a, 0x020f097d, 0x00000000, 0x00000000,   /* G17 */
    0x431efb7a, 0x010f097d, 0x00000000, 0x00000000,   /* G18 */
    0x431efb7a, 0x000f097d, 0x00000000, 0x00000000,   /* G19 */
    0xc31f5b7a, 0x051f097e, 0x00000000, 0x00000000,   /* G20 */
    0xc31f5b7a, 0x041f097e, 0x00000000, 0x00000000,   /* G21 */
    0xc31f5b7a, 0x031f097e, 0x00000000, 0x00000000,   /* G22 */
    0xc31f5b7a, 0x021f097e, 0x00000000, 0x00000000,   /* G23 */
    0xc31f5b7a, 0x011f097e, 0x00000000, 0x00000000,   /* G24 */
    0x429efb7a, 0x0d0f087d, 0x00000002, 0x00000000,   /* G25 */
    0x429efb7a, 0x0c0f087d, 0x00000002, 0x00000000,   /* G26 */
    0x429efb7a, 0x0b0f087d, 0x00000002, 0x00000000,   /* G27 */
    0x429efb7a, 0x0a0f087d, 0x00000002, 0x00000000,   /* G28 */
    0x429efb7a, 0x090f087d, 0x00000002, 0x00000000,   /* G29 */
    0x425efb7a, 0x0d0f087d, 0x00000003, 0x00000000,   /* G30 */
    0x425efb7a, 0x0c0f087d, 0x00000003, 0x00000000,   /* G31 */
    0x425efb7a, 0x0b0f087d, 0x00000003, 0x00000000,   /* G32 */
    0x425efb7a, 0x0a0f087d, 0x00000003, 0x00000000,   /* G33 */
    0x425efb7a, 0x090f087d, 0x00000003, 0x00000000,   /* G34 */
    0x425efb7a, 0x080f087d, 0x00000003, 0x00000000,   /* G35 */
    0xc25f5b7a, 0x0d1f087e, 0x00000003, 0x00000000,   /* G36 */
    0xc25f5b7a, 0x0c1f087e, 0x00000003, 0x00000000,   /* G37 */
    0x41defb7a, 0x150f077d, 0x00000005, 0x00000000,   /* G38 */
    0x41defb7a, 0x140f077d, 0x00000005, 0x00000000,   /* G39 */
    0x41defb7a, 0x130f077d, 0x00000005, 0x00000000,   /* G40 */
    0x41defb7a, 0x120f077d, 0x00000005, 0x00000000,   /* G41 */
    0x41defb7a, 0x110f077d, 0x00000005, 0x00000000,   /* G42 */
    0x411efb7a, 0x150f087d, 0x0000000b, 0x00000000,   /* G43 */
    0x411efb7a, 0x140f087d, 0x0000000b, 0x00000000,   /* G44 */
    0x411efb7a, 0x130f087d, 0x0000000b, 0x00000000,   /* G45 */
    0x411efb7a, 0x120f087d, 0x0000000b, 0x00000000,   /* G46 */
    0x3f9efb7a, 0x1d0f087d, 0x0000000d, 0x00000000,   /* G47 */
    0x3f9efb7a, 0x1c0f087d, 0x0000000d, 0x00000000,   /* G48 */
    0x3f9efb7a, 0x1b0f087d, 0x0000000d, 0x00000000,   /* G49 */
    0x3f9efb7a, 0x1a0f087d, 0x0000000d, 0x00000000,   /* G50 */
    0x3f9efb7a, 0x190f087d, 0x0000000d, 0x00000000,   /* G51 */
    0x3f9efb7a, 0x180f087d, 0x0000000d, 0x00000000,   /* G52 */
    0xbf9f5b7a, 0x1d1f087e, 0x0000000d, 0x00000000,   /* G53 */
    0x3c1efb7a, 0x3d0f067d, 0x0000000f, 0x00000000,   /* G54 */
    0x3c1efb7a, 0x3c0f067d, 0x0000000f, 0x00000000,   /* G55 */
    0x3c1efb7a, 0x3b0f067d, 0x0000000f, 0x00000000,   /* G56 */
    0x3c1efb7a, 0x3a0f067d, 0x0000000f, 0x00000000,   /* G57 */
    0x3c1efb7a, 0x390f067d, 0x0000000f, 0x00000000,   /* G58 */
    0x3c1efb7a, 0x380f067d, 0x0000000f, 0x00000000,   /* G59 */
    0xbc1f5b7a, 0x3d1f067e, 0x0000000f, 0x00000000,   /* G60 */
    0xbc1f5b7a, 0x3c1f067e, 0x0000000f, 0x00000000,   /* G61 */
    0xbc1f5b7a, 0x3b1f067e, 0x0000000f, 0x00000000,   /* G62 */
    0xbc1f5b7a, 0x3a1f067e, 0x0000000f, 0x00000000,   /* G63 */
    0xbc1f5b7a, 0x391f067e, 0x0000000f, 0x00000000,   /* G64 */
    0xbc1f5b7a, 0x381f067e, 0x0000000f, 0x00000000,   /* G65 */
    0xbc1f5b7a, 0x3d3f067e, 0x0000000f, 0x00000000,   /* G66 */
    0xbc1f5b7a, 0x3c3f067e, 0x0000000f, 0x00000000,   /* G67 */
    0xbc1f5b7a, 0x3b3f067e, 0x0000000f, 0x00000000,   /* G68 */
    0xbc1f5b7a, 0x3a3f067e, 0x0000000f, 0x00000000,   /* G69 */
    0xbc1f5b7a, 0x393f067e, 0x0000000f, 0x00000000,   /* G70 */
    0xbc1f5b7a, 0x383f067e, 0x0000000f, 0x00000000,   /* G71 */
    0xbc1f5b7a, 0x3d7f067e, 0x0000000f, 0x00000000,   /* G72 */
    0xbc1f5b7a, 0x3c7f067e, 0x0000000f, 0x00000000,   /* G73 */
    0xbc1f5b7a, 0x3b7f067e, 0x0000000f, 0x00000000,   /* G74 */
    0xbc1f5b7a, 0x3a7f067e, 0x0000000f, 0x00000000,   /* G75 */
    0xbc1f5b7a, 0x397f067e, 0x0000000f, 0x00000000,   /* G76 */
    0xbc1f5b7a, 0x387f067e, 0x0000000f, 0x00000000,   /* G77 */
    0xbc1f5b7a, 0x3dff067e, 0x0000000f, 0x00000000,   /* G78 */
    0xbc1f5b7a, 0x3cff067e, 0x0000000f, 0x00000000,   /* G79 */
    0xbc1f5b7a, 0x3bff067e, 0x0000000f, 0x00000000,   /* G80 */
    0xbc1f5b7a, 0x3aff067e, 0x0000000f, 0x00000000,   /* G81 */
    0xbc1f5b7a, 0x39ff067e, 0x0000000f, 0x00000000,   /* G82 */
    0xbc1f5b7a, 0x38ff067e, 0x0000000f, 0x00000000,   /* G83 */
};

static unsigned s_fail, s_run;
static void check(const char *name, int ok)
{
    ++s_run;
    printf("%-4s %s\n", ok ? "ok" : "FAIL", name);
    s_fail += !ok;
}

int main(void)
{
    arc_gain_table_t t;
    arc_gain_table_from_bytes(&t, NULL, ARC_5G_GAIN_MAX);
    const unsigned n = sizeof(s_board) / sizeof(s_board[0]);
    int bad;
    check("the board's AGC memory equals the reconstruction (84/84)",
          arc_gain_table_check_agc(&t, s_board, n, &bad) == 84u && bad == -1);

    uint16_t rf;
    uint8_t bb, fine;
    arc_gain_entry_from_agc(&s_board[25 * ARC_AGC_ENTRY_WORDS], &rf, &bb, &fine);
    check("G25 decodes to RF 65 (stage 3), BB 7, fine 5", rf == 65u && bb == 7u && fine == 5u);
    arc_gain_entry_from_agc(&s_board[83 * ARC_AGC_ENTRY_WORDS], &rf, &bb, &fine);
    check("G83 decodes to RF 487 (stage 8), BB 127, fine 0", rf == 487u && bb == 127u && fine == 0u);

    uint32_t w[sizeof(s_board) / sizeof(s_board[0])];
    memcpy(w, s_board, sizeof(w));
    w[40 * ARC_AGC_ENTRY_WORDS + 1] ^= 1u << 24;   /* G40 fine code */
    check("a changed fine code is found at its row",
          arc_gain_table_check_agc(&t, w, n, &bad) == 83u && bad == 40);
    memcpy(w, s_board, sizeof(w));
    w[54 * ARC_AGC_ENTRY_WORDS + 2] += 1u;           /* G54 RF code high bits */
    check("a changed RF code is found at its row",
          arc_gain_table_check_agc(&t, w, n, &bad) == 83u && bad == 54);
    check("rows beyond the words read count as missing",
          arc_gain_table_check_agc(&t, s_board, 60u * ARC_AGC_ENTRY_WORDS, &bad) == 60u && bad == 60);
    check("no words: nothing matches", arc_gain_table_check_agc(&t, NULL, 0u, &bad) == 0u);

    /* DC words (arc_gain_entry_dc). The vendor writes one bank-1 pair per RF
     * stage and one bank-2 pair per BB half (BB < 15 / >= 15) of the current
     * frequency group: the board's rows must show exactly that structure. */
    int stage_b1[ARC_RX_STAGE_COUNT][2], half_b2[2][2];
    bool stage_seen[ARC_RX_STAGE_COUNT] = {false}, half_seen[2] = {false, false}, same = true;
    for (unsigned g = 0; g <= ARC_5G_GAIN_MAX; ++g) {
        arc_gain_tuple_t tu;
        int b1[2], b2[2];
        if (!arc_gain_tuple_decode(&t, (uint8_t)g, &tu)) { same = false; continue; }
        arc_gain_entry_dc(&s_board[g * ARC_AGC_ENTRY_WORDS], b1, b2);
        const unsigned s = tu.rf_stage, h = tu.bb_code >= 15u;
        if (!stage_seen[s]) { stage_b1[s][0] = b1[0]; stage_b1[s][1] = b1[1]; stage_seen[s] = true; }
        if (!half_seen[h]) { half_b2[h][0] = b2[0]; half_b2[h][1] = b2[1]; half_seen[h] = true; }
        same &= b1[0] == stage_b1[s][0] && b1[1] == stage_b1[s][1];
        same &= b2[0] == half_b2[h][0] && b2[1] == half_b2[h][1];
    }
    check("board rows: bank 1 per RF stage, bank 2 per BB half", same && half_seen[0] && half_seen[1]);
    int b1[2], b2[2];
    arc_gain_entry_dc(&s_board[83 * ARC_AGC_ENTRY_WORDS], b1, b2);
    check("board G83 DC words 262/240 and 253/250",
          b1[0] == 262 && b1[1] == 240 && b2[0] == 253 && b2[1] == 250);

    /* Vendor packing (phy_wr_rx_gain_mem, 5 GHz): w0 = b2I<<31 | b1Q<<22 |
     * b2Q<<13 | other, w1 = rf[4]<<31 | rf[3]<<30 | rf[2:0]<<27 | fine<<24 |
     * BB<<17 | b1I<<8 | b2I>>1, w2 = rf >> 5. Bank 1 274/231 is the board's
     * G83 on 5880 MHz (2026-10-27); bank 2 I odd puts its LSB in w0[31]. */
    const uint32_t rf_c = 487u, bb_c = 127u, fine_c = 0u;
    const uint32_t i1 = 274u, q1 = 231u, i2 = 225u, q2 = 266u;
    const uint32_t row[ARC_AGC_ENTRY_WORDS] = {
        (i2 & 1u) << 31 | q1 << 22 | q2 << 13 | 0x1abu,
        ((rf_c >> 4) & 1u) << 31 | ((rf_c >> 3) & 1u) << 30 | (rf_c & 7u) << 27 | fine_c << 24 | bb_c << 17 |
            i1 << 8 | i2 >> 1,
        rf_c >> 5, 0u};
    arc_gain_entry_dc(row, b1, b2);
    check("packed row: bank 1 274/231, bank 2 225/266 (odd I)",
          b1[0] == 274 && b1[1] == 231 && b2[0] == 225 && b2[1] == 266);
    arc_gain_entry_from_agc(row, &rf, &bb, &fine);
    check("packed row: RF/BB/fine unaffected by the DC words", rf == rf_c && bb == bb_c && fine == fine_c);
    const uint32_t ones[ARC_AGC_ENTRY_WORDS] = {UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX};
    arc_gain_entry_dc(ones, b1, b2);
    check("all ones: every DC word is 9 bits (511)", b1[0] == 511 && b1[1] == 511 && b2[0] == 511 && b2[1] == 511);

    printf("%u/%u AGC table checks passed\n", s_run - s_fail, s_run);
    return s_fail ? 1 : 0;
}
