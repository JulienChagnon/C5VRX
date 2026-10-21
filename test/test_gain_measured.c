/* C5VRX-5 by Twotoz and the C5VRX contributors: dg_apply_measured_gain
 * (main/gain.c) with the board's measured top rows (PR #228, pair sweep on
 * receiver noise: 1.0 dB a fine step, 0.93 dB at G78/G77).
 *
 *   cc -O2 -std=c11 -I../main test_gain_measured.c ../main/gain.c ../main/arc_phy.c -lm -o t && ./t
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "gain.h"

unsigned char phy_param[2048];
static unsigned s_fail, s_run;
static void check(const char *name, int ok) { ++s_run; printf("%-4s %s\n", ok ? "ok" : "FAIL", name); s_fail += !ok; }

int main(void)
{
    arc_gain_table_t table;
    arc_gain_table_from_bytes(&table, NULL, ARC_5G_GAIN_MAX);
    static direct_gain_t g;
    dg_reset(&g, &table, 83);
    const int16_t model83 = g.model_db_q8[83], model72 = g.model_db_q8[72], model40 = g.model_db_q8[40],
                  model71 = g.model_db_q8[71];
    float th[DG_STATES] = {0};
    /* measured: 1.0 dB a row from G83 down to G72, except 0.93 at G78/77 */
    double db = 0.0;
    th[83] = 7.1f;
    for (int k = 82; k >= 72; --k) { db -= (k == 77 ? 0.93 : 1.0); th[k] = (float)(7.1 * pow(10.0, db / 10.0)); }
    th[75] = 0.0f;   /* one row missing: interpolated */
    const unsigned set = dg_apply_measured_gain(&g, th);
    const double d72 = (g.model_db_q8[83] - g.model_db_q8[72]) / 256.0;
    printf("     rows set %u; G83-G72 %.2f dB (model %.2f); G75 %.2f dB under G83\n", set, d72,
           (model83 - model72) / 256.0, (g.model_db_q8[83] - g.model_db_q8[75]) / 256.0);
    check("measured rows take the measured gain (G83-G72 = 10.93 dB)", set == 11u && fabs(d72 - 10.93) < 0.02);
    check("the top row keeps its model value", g.model_db_q8[83] == model83);
    check("an unmeasured row between measured ones is interpolated (G75 ~8 dB under G83)",
          fabs((g.model_db_q8[83] - g.model_db_q8[75]) / 256.0 - 7.93) < 0.3);
    (void)model71; (void)model40;
    check("rows below step 1.0 dB a row (measured on the board), joined to the lowest measured row",
          g.model_db_q8[72] - g.model_db_q8[71] == 256 && g.model_db_q8[72] - g.model_db_q8[40] == 32 * 256);
    /* home rows 3 dB apart on the measured gains */
    bool spaced = g.keel_rows >= 3u;
    for (unsigned k = 1; k < g.keel_rows && g.keel_row[k] >= 72u; ++k) {
        const double gap = (g.model_db_q8[g.keel_row[k]] - g.model_db_q8[g.keel_row[k - 1]]) / 256.0;
        spaced = spaced && gap > 2.4 && gap < 3.6;
    }
    check("the home rows are rebuilt ~3 dB apart on the measured gains", spaced);
    printf("%u/%u measured gain checks passed\n", s_run - s_fail, s_run);
    return s_fail != 0;
}
