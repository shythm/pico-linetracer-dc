#ifndef __TIMER_H_
#define __TIMER_H_

/*! \brief 다른 코드의 진행 여부에 상관 없이 일정한 주기로 특정 함수를 실행할 수 있도록 도와주는 함수
 * \param num 타이머 슬롯 (0 ~ 3)
 * \param interval 타이머 주기 (단위: microsecond(us))
 * \param handler 주기적으로 실행할 함수(핸들러)
 */
void timer_periodic_start(uint num, uint interval, void (*handler)(void));

/*! \brief 주기적 타이머 멈춤 함수
 * \param num 타이머 슬롯 (0 ~ 3)
 */
void timer_periodic_stop(uint num);

#endif