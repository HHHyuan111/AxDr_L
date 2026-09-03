#include "common.h"


void nlob_init(void)
{
	memset(&g_foc.nlob, 0, sizeof(g_foc.nlob));
	
	g_foc.nlob.i_alph = &g_foc.sig.i_alph;
	g_foc.nlob.i_beta = &g_foc.sig.i_beta;
	
	g_foc.nlob.v_alph = &g_foc.sig.v_alph;
	g_foc.nlob.v_beta = &g_foc.sig.v_beta;

	g_foc.nlob.i_q  = &g_foc.sig.i_q;
	
	g_foc.nlob.Rs = &g_foc.motor.Rs;
	g_foc.nlob.Ls = &g_foc.motor.Ls;
	g_foc.nlob.flux = &g_foc.motor.flux;
	
	g_foc.nlob.flux_sqr  = g_foc.nlob.flux[0] * g_foc.nlob.flux[0];
	g_foc.nlob.bw_factor = 1.0f / g_foc.nlob.flux_sqr;
	g_foc.nlob.gain  = 1000.0f;
	g_foc.nlob.gamma = 0.5f * (g_foc.nlob.gain * g_foc.nlob.bw_factor);
	g_foc.nlob.id_gain = 1.0f;
	g_foc.nlob.ts = 0.00005f;
	g_foc.nlob.fs = 20000;
	
	g_foc.nlob.x1 = g_foc.nlob.flux[0];
	g_foc.nlob.x2 = 0;
	
    g_foc.nlob.pll.wn = 100 * 2 * AXDR_PI;  // 带宽
	g_foc.nlob.pll.damp = 0.707f; 		// 阻尼系数
	g_foc.nlob.pll.ts   = 0.00005f;
	
	g_foc.nlob.pll.kp = 2*g_foc.nlob.pll.damp*g_foc.nlob.pll.wn;
	g_foc.nlob.pll.ki = g_foc.nlob.pll.wn*g_foc.nlob.pll.wn;
    g_foc.nlob.pll.i_term_max = 333 * 2 * AXDR_PI * 2;
    g_foc.nlob.pll.out_max = 333 * 2 * AXDR_PI * 2;
}



_RAM_FUNC void nlob_vesc(nlob_t *obj)
{	
	float sign_we = obj->we > 0 ? 1 : -1; 
    float abs_wc = fabsf(obj->we);
	
	if(abs_wc > 500)
    {
        obj->id_out = 0;
    }
    else
    {
        obj->id_ref = obj->i_q[0]*obj->id_gain*sign_we;
        obj->id_out = 1.5f;//fabsf(obj->id_out * 0.999f + obj->id_ref * 0.001f);
    }
	
	
	float R_ia = obj->Rs[0] * obj->i_alph_lst;
	float L_ia = obj->Ls[0] * obj->i_alph[0];
	
	float R_ib = obj->Rs[0] * obj->i_beta_lst;
	float L_ib = obj->Ls[0] * obj->i_beta[0];
	
	float err = obj->flux_sqr - obj->flux_est_s;
	float x1_dot =  obj->v_alph_lst - R_ia +  obj->gamma * obj->flux_alph * err;
	float x2_dot =  obj->v_beta_lst - R_ib +  obj->gamma * obj->flux_beta * err;
	
	obj->x1 += x1_dot * obj->ts;
	obj->x2 += x2_dot * obj->ts;

	obj->flux_alph = obj->x1 - L_ia;
	obj->flux_beta = obj->x2 - L_ib;
	obj->flux_est_s= SQ(obj->flux_alph) + SQ(obj->flux_beta);
	obj->flux_est  = sqrtf(obj->flux_est_s);

	obj->flux_err = *obj->flux - obj->flux_est;

#if 1
	// atan
	obj->pos_e = atan2f(obj->flux_beta, obj->flux_alph);
	wrap_0_2pi(obj->pos_e);
	pll_calc(&obj->pll, obj->pos_e);
#endif

#if 0
	/* Orthogonal PLL */
	ort_pll_calc(&obj->pll, obj->flux_alph, obj->flux_beta, obj->flux[0]);
#endif
	
	// 更新上一次的 i_alpha, i_beta, v_alpha, v_beta
    obj->i_alph_lst = obj->i_alph[0];
    obj->i_beta_lst = obj->i_beta[0];
    obj->v_alph_lst = obj->v_alph[0];
    obj->v_beta_lst = obj->v_beta[0];
}
