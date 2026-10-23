/* C5VRX by Twotoz/contributors: kp confirmation and shared restart guard. */
#include <assert.h>
#include <stdio.h>
#include "kp_switch_guard.h"
int main(void) {
    kp_switch_guard_t physical={0}, candidate={0};
    assert(kp_switch_ready(&physical,0));
    kp_switch_mark(&physical,100);
    assert(!kp_switch_ready(&physical,100+KP_SWITCH_GAP_US-1));
    assert(kp_switch_ready(&physical,100+KP_SWITCH_GAP_US));
    assert(!kp_switch_worth(3,2,100,1000));
    assert(kp_switch_worth(4,2,100,1000));
    assert(kp_switch_worth(3,2,100,100+KP_ONE_LEVEL_GAP_US));
    assert(!kp_switch_confirm(&candidate,3,7,true,0));
    assert(!kp_switch_confirm(&candidate,3,7,true,KP_SWITCH_CONFIRM_US-1));
    assert(kp_switch_confirm(&candidate,3,7,true,KP_SWITCH_CONFIRM_US));
    assert(!kp_switch_confirm(&candidate,3,8,true,KP_SWITCH_CONFIRM_US+1));
    candidate=(kp_switch_guard_t){0};
    /* Moving farther in the same direction must not restart confirmation. */
    for(int64_t t=0;t<=KP_SWITCH_CONFIRM_US;t+=100000) {
        bool ready=kp_switch_confirm_toward(&candidate,t<1000000?3:4,2,1,true,t);
        assert(ready==(t>=KP_SWITCH_CONFIRM_US));
    }
    assert(!kp_switch_confirm_toward(&candidate,1,2,1,true,2100000));
    assert(!kp_switch_confirm_toward(&candidate,1,2,1,false,2200000));
    assert(!kp_switch_confirm_toward(&candidate,1,2,1,true,2800000));
    assert(candidate.candidate_since_us==2800000);
    assert(!kp_switch_confirm_toward(&candidate,2,2,1,true,2900000));
    assert(!candidate.candidate);
    puts("PASS: cooldown, step value, direction, stale evidence and context changes");
}
