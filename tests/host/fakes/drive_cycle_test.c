#include <stdlib.h>
#include "common.h"
#include "control_cycle.h"
#include "board_adapter.h"
#include "position_adapter.h"
#include "drive_cycle_test.h"

void run_drive_cycle(foc_t *foc)
{
    const control_cycle_input_t input = {
        .seq=foc->fast_seq,
        .i_valid=foc->fb.i_valid, .vbus_valid=foc->fb.vbus_valid, .pos_valid=foc->fb.pos_valid,
        .ia=foc->fb.ia, .ib=foc->fb.ib, .ic=foc->fb.ic, .vbus=foc->fb.vbus,
        .theta_e=foc->fb.theta_e, .pos_r=foc->fb.pos_r, .pos_m=foc->fb.pos_m,
    };
    control_cycle_step(foc,&input,NULL);
}

void ctrl_fb_update(foc_t *foc, float vbus) { foc->fb.vbus=vbus; }
void obs_step(const foc_t *foc) { (void)foc; }
bool encoder_sample(encoder_state_t *e) { (void)e; abort(); }
bool position_update(foc_t *f, position_sample_t *p) { (void)f; (void)p; abort(); }
bool foc_adc_sample(foc_t *f, board_sample_t *p) { (void)f; (void)p; abort(); }
