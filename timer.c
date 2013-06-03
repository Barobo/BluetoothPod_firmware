#include <stdio.h>
#include "timer.h"

volatile timer_t *g_timerHead = NULL;

void timerStart(timer_t* timer)
{
  /* Make sure the timer is stopped first */
  timerStop(timer);
  /* Set the time_millis member */
  timer->time_millis = g_millis + timer->interval_millis;
  timer->next = NULL;
  timer->prev = NULL;
  /* Insert the timer into the correct location */
  timer_t* iter;
  timer_t* prev = NULL;
  
  if(g_timerHead != NULL && (timer->time_millis < g_timerHead->time_millis)) {
    timer->next = g_timerHead;
    g_timerHead->prev = timer;
    g_timerHead = timer;
    return;
  }

  for(iter = g_timerHead; (iter != NULL) && (timer->time_millis >= iter->time_millis) ; iter = iter->next) {
    prev = iter;
  }

  /* If we get here, the iter time must by higher than our timer time, or iter
   * is null, so we must add our timer before iter and after "prev"  */
  if(prev != NULL) {
    prev->next = timer;
  }
  timer->prev = prev;

  timer->next = iter;
  if(iter != NULL) {
    iter->prev = timer;
  }
  /* Need to attach head if unattached */
  if(g_timerHead == NULL) {
    g_timerHead = timer;
  }
}

void timerStop(timer_t* timer)
{
  if(timer == g_timerHead) {
    g_timerHead = timer->next;
    if(timer->next != NULL) {
      timer->next->prev = NULL;
    }
    timer->next = NULL;
    timer->prev = NULL;
    return;
  }

  if(timer->prev != NULL) {
    timer->prev->next = timer->next;
  }
  if(timer->next != NULL) {
    timer->next->prev = timer->prev;
  }
  timer->next = NULL;
  timer->prev = NULL;
}

void timerHandler()
{
  timer_t* tmp;
  /*
  char buf[80];
  if(g_timerHead != NULL) {
    sprintf(buf, "%lu %lu\r\n", g_timerHead->time_millis, g_millis);
    serialWriteString(buf);
  }
  */
  if ((g_timerHead != NULL) && (g_timerHead->time_millis <= g_millis))
  {
    tmp = g_timerHead;
    /* Remove the iter from the list first */
    timerStop(g_timerHead);
    /* Call the callback */
    tmp->callback();
    /* Reinsert into the list if periodic handler */
    if(tmp->mode == TIMER_PERIODIC) {
      timerStart(tmp);
    }
  }
}
