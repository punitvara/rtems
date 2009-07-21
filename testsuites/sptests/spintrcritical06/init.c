/*
 *  COPYRIGHT (c) 1989-2009.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *  $Id$
 */

#define __RTEMS_VIOLATE_KERNEL_VISIBILITY__ 1
#include <tmacros.h>
#include <intrcritical.h>

/* common parameters */
#define SEMAPHORE_ATTRIBUTES     RTEMS_PRIORITY
#define SEMAPHORE_OBTAIN_TIMEOUT 0

#if defined(PRIORITY_NO_TIMEOUT_FORWARD)
  #define TEST_NAME          "06"
  #define TEST_STRING        "Priority/Restart Search Task (Forward)"

  #define INIT_PRIORITY      2
  #define BLOCKER_PRIORITY   1

#elif defined(PRIORITY_NO_TIMEOUT_REVERSE)
  #define TEST_NAME          "07"
  #define TEST_STRING        "Priority/Restart Search Task (Backward)"
  #define INIT_PRIORITY      126
  #define BLOCKER_PRIORITY   127

#else

  #error "Test Mode not defined"
#endif

rtems_id Main_task;
rtems_id Secondary_task_id;
rtems_id Semaphore;
volatile bool case_hit;

Thread_blocking_operation_States getState(void)
{
  Objects_Locations  location;
  Semaphore_Control *sem;

  sem = (Semaphore_Control *)_Objects_Get(
    &_Semaphore_Information, Semaphore, &location ); 
  if ( location != OBJECTS_LOCAL ) {
    puts( "Bad object lookup" );
    rtems_test_exit(0);
  }
  _Thread_Unnest_dispatch();

  return sem->Core_control.semaphore.Wait_queue.sync_state;
}

rtems_timer_service_routine test_release_from_isr(
  rtems_id  timer,
  void     *arg
)
{
printk("r");
  (void) rtems_task_restart( Secondary_task_id, 1 );
}

rtems_task Secondary_task(
  rtems_task_argument arg
)
{
  rtems_status_code     status;

#if 0
  if ( arg ) {
    printk("f");
    (void) rtems_semaphore_flush( Semaphore );
  }
#endif

  #if 0 && defined(PRIORITY_NO_TIMEOUT_REVERSE)
    status = rtems_task_resume( Main_task );
    directive_failed( status, "rtems_task_resume" );
  #endif

    printk("O");
  status = rtems_semaphore_obtain(
    Semaphore,
    RTEMS_DEFAULT_OPTIONS,
    SEMAPHORE_OBTAIN_TIMEOUT
  );
  directive_failed( status, "rtems_semaphore_obtain" );
}

rtems_task Init(
  rtems_task_argument ignored
)
{
  rtems_status_code     status;
  int                   resets;

  puts( "\n\n*** TEST INTERRUPT CRITICAL SECTION " TEST_NAME " ***" );

#if defined(PRIORITY_NO_TIMEOUT_REVERSE)
  puts( "WARNING!!! TEST IS NOT COMPLETE!!!" );
  puts( "WARNING!!! TEST IS NOT COMPLETE!!!" );
  puts( "WARNING!!! TEST IS NOT COMPLETE!!!" );
  puts( "WARNING!!! TEST IS NOT COMPLETE!!!" );
#endif

  puts( "Init - Trying to generate semaphore release from ISR while blocking" );
  puts( "Init - Variation is: " TEST_STRING );
  status = rtems_semaphore_create(
    rtems_build_name( 'S', 'M', '1', ' ' ),
    0,
    SEMAPHORE_ATTRIBUTES,
    RTEMS_NO_PRIORITY,
    &Semaphore
  );
  directive_failed( status, "rtems_semaphore_create of SM1" );

  Main_task = rtems_task_self();

  status = rtems_task_create(
    rtems_build_name( 'B', 'L', 'C', 'K' ),
    1,
    RTEMS_MINIMUM_STACK_SIZE,
    RTEMS_NO_PREEMPT,
    RTEMS_DEFAULT_ATTRIBUTES,
    &Secondary_task_id
  );
  directive_failed( status, "rtems_task_create" );

  status = rtems_task_start( Secondary_task_id, Secondary_task, 0 );
  directive_failed( status, "rtems_task_start" );

  interrupt_critical_section_test_support_initialize( test_release_from_isr );

  for (resets=0 ; resets< 2 ;) {
    if ( interrupt_critical_section_test_support_delay() )
      resets++;
    #if defined(PRIORITY_NO_TIMEOUT_REVERSE)
      status = rtems_task_suspend( RTEMS_SELF );
      directive_failed( status, "rtems_task_suspend" );
    #endif

printk("o");
    status = rtems_semaphore_obtain(
      Semaphore,
      RTEMS_DEFAULT_OPTIONS,
      SEMAPHORE_OBTAIN_TIMEOUT
    );
    fatal_directive_status(status, RTEMS_UNSATISFIED, "rtems_semaphore_obtain");
  }

  puts( "*** END OF TEST INTERRUPT CRITICAL SECTION " TEST_NAME " ***" );
  rtems_test_exit(0);
}

/* configuration information */

#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER

#define CONFIGURE_MAXIMUM_TASKS       2
#define CONFIGURE_MAXIMUM_TIMERS      1
#define CONFIGURE_MAXIMUM_SEMAPHORES  1
#define CONFIGURE_INIT_TASK_PRIORITY  INIT_PRIORITY
#define CONFIGURE_INIT_TASK_MODE      RTEMS_PREEMPT
#define CONFIGURE_RTEMS_INIT_TASKS_TABLE

#define CONFIGURE_INIT
#include <rtems/confdefs.h>

/* global variables */
