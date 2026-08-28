#ifndef TARGET_PWM_H
#define TARGET_PWM_H

void target_pwm_start_phase_outputs(void);
void target_pwm_stop_phase_outputs(void);
void target_pwm_commit_channel_duty(float channel_1_duty,
                                    float channel_2_duty,
                                    float channel_3_duty);

#endif /* TARGET_PWM_H */
