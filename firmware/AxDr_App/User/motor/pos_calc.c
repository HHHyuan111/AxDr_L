#include "common.h"

/**
 * @brief 按当前位置反馈配置读取本控制周期的编码器原始值。
 *
 * @param[in,out] pos_box 位置反馈配置和各编码器数据对象。
 *
 * 单编码器模式读取主编码器；双编码器模式依次读取主、副编码器；无传感器
 * 模式不读取物理编码器。本函数只负责采样，不计算机械角和电角度。
 */
_RAM_FUNC void encoder_sample(pos_box_t *pos_box)
{
    switch (pos_box->pos_mode) {
        case Sensorsory_s:
            switch (pos_box->sensory1) {
                case MA732:  read_ma732_raw(); break;
                case MT6816: read_mt6816_raw(&pos_box->mt6816); break;
                case MT6825: read_mt6825_raw(); break;
                case DMENC:  read_dm485enc_raw(); break; // 如果是DM485编码器
                default:     break;
            }
            break;
        case Sensorsory_d:
            switch (pos_box->sensory1) {
                case MA732:  read_ma732_raw(); break;
                case MT6816: read_mt6816_raw(&pos_box->mt6816); break;
                case MT6825: read_mt6825_raw(); break;
                case DMENC:  read_dm485enc_raw(); break; // 如果是DM485编码器
                default:     break;
            }
            switch (pos_box->sensory2) {
                case MA732:  read_ma732_raw(); break;   // 如有第二路SPI/IO
                case MT6816: read_mt6816_raw(&pos_box->mt6816); break;
                case MT6825: read_mt6825_raw(); break;
                default:     break;
            }
            break;
        case Sensorless:
            /* 无传感器模式不读取物理编码器。 */
            break;
        default:
            break;
    }
}

/**
 * @brief 根据当前位置模式更新机械角、电角度和多圈位置反馈。
 *
 * @param[in,out] pm 电机控制对象，提供位置配置并接收本周期的位置结果。
 *
 * 有感模式使用 encoder_sample 已取得的编码器原始值；无传感器模式使用选定
 * 观测器的输出。本函数不直接访问 SPI 或编码器片选引脚。
 */
_RAM_FUNC void position_update(pmsm_t *pm)
{
    enc_para_t *primary_enc = NULL;
    enc_para_t *secondary_enc = NULL;

    /* 根据位置模式选择本周期使用的位置来源。 */
    switch (pm->pos_box.pos_mode)
    {
        case Sensorsory_s: // 单编码器有感
            switch (pm->pos_box.sensory1) {
                case MT6825: primary_enc = &pm->pos_box.mt6825;   break;
                case MT6816: primary_enc = &pm->pos_box.mt6816;   break;
                case MA732:  primary_enc = &pm->pos_box.ma732;    break;
                case DMENC:  primary_enc = &pm->pos_box.dm485enc; break;
                default: break;
            }

            if (primary_enc->rev_flag) {
                primary_enc->rev_flag = 0;
                pos_encoder_calc(primary_enc);
                pm->foc.e_pr = primary_enc->pos; // 主编码器结果
                position_update_single_encoder(pm);

                pm->pos_box.raw_1 = primary_enc->raw;
                pm->pos_box.bit_1 = primary_enc->bit;
                pm->pos_box.pos_1 = primary_enc->pos;
            }
            break;

        case Sensorsory_d: // 双编码器有感
            switch (pm->pos_box.sensory1) {
                case MT6825: primary_enc = &pm->pos_box.mt6825;   break;
                case MT6816: primary_enc = &pm->pos_box.mt6816;   break;
                case MA732:  primary_enc = &pm->pos_box.ma732;    break;
                case DMENC:  primary_enc = &pm->pos_box.dm485enc; break;
                default: break;
            }
            switch (pm->pos_box.sensory2) {
                case MT6825: secondary_enc = &pm->pos_box.mt6825; break; // 如有独立enc_para_t建议新建
                case MT6816: secondary_enc = &pm->pos_box.mt6816; break;
                case MA732:  secondary_enc = &pm->pos_box.ma732;  break;
                default: break;
            }

            if (primary_enc->rev_flag && secondary_enc->rev_flag) {
                primary_enc->rev_flag = 0;
                secondary_enc->rev_flag = 0;
                pos_encoder_calc(secondary_enc);
                pm->foc.e_pr = primary_enc->pos; // 转子侧用主编码器
                pm->foc.e_pm = secondary_enc->pos; // 机械输出轴侧角度用副编码器
                sensory2_pos_calc(pm);

                pm->pos_box.raw_1 = primary_enc->raw;
                pm->pos_box.bit_1 = primary_enc->bit;
                pm->pos_box.pos_1 = primary_enc->pos;
                pm->pos_box.raw_2 = secondary_enc->raw;
                pm->pos_box.bit_2 = secondary_enc->bit;
                pm->pos_box.pos_2 = secondary_enc->pos;
            }
            break;

        case Sensorless:
            switch (pm->pos_box.senless)
            {
                case Nlob:
                    nlob_vesc(&pm->nlob);
                    pm->foc.p_e = pm->nlob.pos_e;
                    pm->ctrl.id_set = pm->nlob.id_out;
                    break;
                case Alob:
                    alob_flux(&pm->alob);
                    pm->foc.p_e = pm->alob.pos_e;
                    pm->ctrl.id_set = pm->alob.id_out;
                    break;
                case Scvm:
                    scvm_obe(&pm->scvm);
                    pm->foc.p_e = pm->scvm.pos_e;
                    pm->ctrl.id_set = pm->scvm.id_out;
                    break;
                case Esmo:
                    break;
                case Hsfi:
                    break;
                case Ekf:
                    break;
                default:
                    return; // 不支持的编码器类型
            }
            senless_pos_calc(pm);
            break;
        default: return; // 不支持的角度类型
    }
}

/**
 * @brief 根据主编码器单圈角度更新电机的完整位置反馈。
 *
 * @param[in,out] pm 电机控制对象；输入主编码器角度和电机参数，输出电角度、
 *                   转子单圈/多圈位置、输出轴单圈/多圈位置及机械圈数。
 *
 * 本函数不读取编码器硬件。进入本函数前，pm->foc.e_pr 已经是 0～2π 范围内
 * 的主编码器机械角度。
 */
_RAM_FUNC void position_update_single_encoder(pmsm_t *pm)
{
    pmsm_foc_t *foc = &pm->foc;
    pmsm_para_t *motor_para = &pm->para;

    /*
     * 第 1 步：判断单圈角度是否跨过 0/2π 边界，并累计转子圈数。
     * 正向越过 2π 时，新角度会从接近 2π 跳到接近 0，差值为较大的负数；
     * 反向越过 0 时，差值为较大的正数。0.88 保留原工程的判断阈值。
     */
    foc->pr_dif = foc->e_pr - foc->pr_lst;
    if (foc->pr_dif > 0.88f * M_2PI) {
        foc->rev--;
    }
    if (foc->pr_dif < -0.88f * M_2PI) {
        foc->rev++;
    }

    /*
     * 第 2 步：机械角乘极对数得到电角度，去掉完整电周期后加电角零位。
     * e_pe 和 p_e 当前保留原工程的相同计算结果，后续控制使用 p_e。
     */
    foc->e_pe = foc->e_pr * motor_para->pn
        - (uint32_t)(foc->e_pr * motor_para->pnd_2pi) * M_2PI
        + motor_para->e_off;
    wrap_0_2pi(foc->e_pe);

    foc->p_e = foc->e_pr * motor_para->pn
        - (uint32_t)(foc->e_pr * motor_para->pnd_2pi) * M_2PI
        + motor_para->e_off;
    wrap_0_2pi(foc->p_e);

    /* 第 3 步：加入转子机械零位，得到转子单圈位置和累计多圈位置。 */
    foc->sp_r = foc->e_pr + motor_para->r_off;
    wrap_0_2pi(foc->sp_r);
    foc->mp_r = foc->e_pr + foc->rev * M_2PI + motor_para->r_off;

    /* 第 4 步：经过减速比换算，得到输出轴单圈位置和累计多圈位置。 */
    foc->sp_m = foc->sp_r * motor_para->div_Gr + motor_para->m_off;
    wrap_0_2pi(foc->sp_m);
    foc->mp_m = foc->mp_r * motor_para->div_Gr;

    /* 第 5 步：把输出轴累计角度换算成完整机械圈数。 */
    foc->m_rev = (int32_t)(foc->mp_m * div_M_2PI);

    /* 保存本周期编码器角度，供下一个周期判断是否跨零。 */
    foc->pr_lst = foc->e_pr;
}

_RAM_FUNC void sensory2_pos_calc(pmsm_t* pm)
{
    pmsm_foc_t* x = &pm->foc;
    pmsm_para_t* y = &pm->para;

    // 计算位置差异和转数
    x->pr_dif = x->e_pr - x->pr_lst;

    if (x->pr_dif >  0.8f * M_2PI) x->rev--;
    if (x->pr_dif < -0.8f * M_2PI) x->rev++;

    // 计算电气角度
    x->e_pe = x->e_pr * y->pn - (uint32_t)(x->e_pr * y->pnd_2pi) * M_2PI;
    wrap_0_2pi(x->e_pe);
    x->p_e  = x->e_pr * y->pn - (uint32_t)(x->e_pr * y->pnd_2pi) * M_2PI + y->e_off;
    wrap_0_2pi(x->p_e);

    x->sp_r = x->e_pr + y->r_off;
    wrap_0_2pi(x->sp_r);
    x->mp_r = x->e_pr + x->rev * M_2PI + y->r_off;

    // 计算减速后单圈、多圈机械角度
    // 计算位置差异和转数
    x->pm_dif = x->e_pm - x->pm_lst;

    if (x->pm_dif >  0.8f * M_2PI) x->m_rev--;
    if (x->pm_dif < -0.8f * M_2PI) x->m_rev++;

    x->sp_m = x->e_pm + y->m_off;
    wrap_0_2pi(x->sp_m);
    x->mp_m = x->sp_m + x->m_rev * M_2PI;

    x->pr_lst = x->e_pr;
    x->pm_lst = x->e_pm;
}

/**
***********************************************************************
* @brief:      senless_pos_calc(pmsm_t* pm)
* @param[in]:  pm  指向永磁同步电机（PMSM）控制结构体的指针
* @retval:     void
* @details:    无传感器模式下的位置和角度计算，通过电角度反推转子角度，计算多圈机械角度和圈数
***********************************************************************
**/
_RAM_FUNC void senless_pos_calc(pmsm_t* pm)
{
    pmsm_foc_t* x = &pm->foc;
    pmsm_para_t* y = &pm->para;

    // 由电角度反推转子角度
    x->sp_r = x->p_e * y->div_pn;
    wrap_0_2pi(x->sp_r);

    // 计算位置差异和转数
    x->pr_dif = x->sp_r - x->pr_lst;
    if (x->pr_dif >  0.8f * M_2PI) x->rev--;
    if (x->pr_dif < -0.8f * M_2PI) x->rev++;

    wrap_0_2pi(x->sp_r);

    // 多圈转子角度
    x->mp_r = x->sp_r + x->rev * M_2PI;

    // 计算减速后单圈、多圈机械角度
    x->sp_m = x->sp_r * y->div_Gr;
    wrap_0_2pi(x->sp_m);
    x->mp_m = x->mp_r * y->div_Gr;

    // 计算减速后机械圈数
    x->m_rev = (int32_t)(x->mp_m / M_2PI);

    x->pr_lst = x->sp_r;
}
