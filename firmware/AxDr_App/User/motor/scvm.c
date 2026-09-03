#include "common.h"

void scvm_init(void)
{
	memset(&g_foc.scvm, 0, sizeof(g_foc.scvm));

	g_foc.scvm.v_alph = &g_foc.sig.v_alph;
	g_foc.scvm.v_beta = &g_foc.sig.v_beta;
	g_foc.scvm.i_alph = &g_foc.sig.i_alph;
	g_foc.scvm.i_beta = &g_foc.sig.i_beta;

	g_foc.scvm.Rs = &g_foc.motor.Rs;
	g_foc.scvm.Ls = &g_foc.motor.Ls;
	g_foc.scvm.Ld = &g_foc.motor.Ld;
	g_foc.scvm.Lq = &g_foc.motor.Lq;
	g_foc.scvm.flux = &g_foc.motor.flux;

	g_foc.scvm.alpha0 = 1500;
	g_foc.scvm.lamda1 = 0.99f;
	g_foc.scvm.id_gain = 0.8f; //0-2之间 越小越稳定，越大低速性能越强
	g_foc.scvm.ts = 0.00005f;
	g_foc.scvm.fs = 20000;
}


_RAM_FUNC void scvm_obe(scvm_t *obj)
{
	float sign_we = obj->we > 0 ? 1 : -1; 
    float abs_wc = fabsf(obj->we);
	
	if(abs_wc > 500)
    {
        obj->id_out = 0;
    }
    else
    {
        obj->id_ref = obj->i_q*obj->id_gain*sign_we;
        obj->id_out = obj->id_out * 0.999f + obj->id_ref * 0.001f;
    }
	
	float sin_val = sin_f32(obj->pos_e);
    float cos_val = cos_f32(obj->pos_e);
	
	obj->e_d = obj->v_d - obj->Rs[0]*obj->i_d + obj->we*obj->Ls[0]*obj->i_q;
    obj->e_q = obj->v_q - obj->Rs[0]*obj->i_q - obj->we*obj->Ls[0]*obj->i_d;
	
	float alpha = obj->alpha0 + 2.0f*obj->lamda1 * abs_wc;
    float wr = (obj->e_q - obj->e_d*sign_we*obj->lamda1)/obj->flux[0];
    float delta_w = alpha*(wr - obj->we);

    obj->we += delta_w*obj->ts;
    obj->pos_e += obj->we*obj->ts;
	wrap_0_2pi(obj->pos_e);
	
	// calc park
    obj->v_d = cos_val*obj->v_alph[0] + sin_val*obj->v_beta[0];
    obj->v_q = cos_val*obj->v_beta[0] - sin_val*obj->v_alph[0];
	
	// calc park
    obj->i_d = cos_val*obj->i_alph[0] + sin_val*obj->i_beta[0];
    obj->i_q = cos_val*obj->i_beta[0] - sin_val*obj->i_alph[0];
	
}



