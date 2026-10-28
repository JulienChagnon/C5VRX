#!/usr/bin/env python3
"""C5VRX/Twotoz: execute actual PHY ownership and gain invalidation ordering."""
from pathlib import Path
import os
import re
import shlex
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SDK = r'''
#undef NDEBUG
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define portMUX_TYPE int
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(p) ((void)(p))
#define portEXIT_CRITICAL(p) ((void)(p))
#define portMAX_DELAY 100
#define pdTRUE 1
typedef int StaticSemaphore_t;
typedef void *SemaphoreHandle_t;
static unsigned locks,hooks,writes;
static bool notified,held=true;
static uint8_t s_gain=50;
static void (*s_post_gain_hook)(uint8_t);
static void *xSemaphoreCreateRecursiveMutexStatic(int *p){return p;}
static int xSemaphoreTakeRecursive(void *p,int timeout){(void)timeout;assert(p);++locks;return 1;}
static int xSemaphoreGiveRecursive(void *p){assert(p&&locks);--locks;return 1;}
static bool phy_dco_held(void){return held;}
static bool phy_dco_release(void){assert(locks&&notified);held=false;return true;}
static void phy_force_rx_gain(bool force,uint8_t gain){assert(force&&gain&&locks&&notified);++writes;}
'''
HARNESS = r'''
static void invalidate(void){assert(locks);++hooks;notified=true;}
int main(void){
 phy_set_observation_hook(invalidate);
 phy_read_begin();phy_read_begin();
 assert(locks==2&&!hooks&&!phy_busy()&&phy_generation()==0);
 phy_read_end();phy_read_end();assert(!locks);
 phy_begin();assert(hooks==1&&phy_busy());
 phy_read_begin();phy_read_end();assert(hooks==1&&phy_busy()&&phy_generation()==0);
 phy_begin();assert(hooks==1);phy_end();assert(phy_busy());
 phy_end();assert(!phy_busy()&&phy_generation()==1&&!locks);
 notified=false;
 assert(phy_try_actuator(1)&&hooks==1);phy_end_actuator();
 assert(!phy_try_actuator(0)&&!locks&&hooks==1);
 assert(rf_set_gain(50,1)&&hooks==1&&!writes); /* same held state: no mutation */
 assert(rf_set_gain(51,1)&&hooks==2&&writes==1&&s_gain==51&&!locks);
 notified=false;phy_set_observation_hook(NULL);
 phy_begin();phy_end();assert(hooks==2&&!locks);
 puts("PASS: PHY outer/nested ownership and pre-gain observation invalidation");
}
'''


class Ordering(unittest.TestCase):
    def test_production_ownership_and_gain(self):
        phy = (ROOT / "main/phy.c").read_text().split("#ifdef C5VRX_PHY_PINNED", 1)[0]
        phy = re.sub(r"^#include[^\n]*\n", "", phy, flags=re.MULTILINE)
        rf = (ROOT / "main/rf.c").read_text()
        gain = rf[rf.index("bool rf_set_gain("):rf.index("void rf_force_gain(")]
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            source = base / "order.c"
            source.write_text(SDK + phy + gain + HARNESS)
            subprocess.run(shlex.split(os.environ.get("CC", "cc")) + ["-O2", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            str(source), "-o", str(base / "order")], check=True)
            subprocess.run([str(base / "order")], check=True)


if __name__ == "__main__":
    unittest.main()
