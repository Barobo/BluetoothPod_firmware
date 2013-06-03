#ifndef _TIMER_H_
#define _TIMER_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum timerMode_e {
  TIMER_PERIODIC,
  TIMER_INTERVAL,
}timerMode_t;

typedef struct timer_s
{
  /* public */
  timerMode_t mode;
  unsigned long interval_millis;
  void (*callback)();

  /* private */
  unsigned long time_millis;

  struct timer_s *next;
  struct timer_s *prev;
} timer_t;

/* public */
void timerStart(timer_t* timer);
void timerStop(timer_t* timer);

/* private */
void timerHandler();

extern volatile timer_t *g_timerHead;
extern volatile unsigned long g_millis;

#ifdef __cplusplus
}
#endif

#endif
