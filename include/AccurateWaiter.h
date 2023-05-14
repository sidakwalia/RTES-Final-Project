//
// Created by jamie on 11/12/2020.
//

#ifndef LIGHTSPEEDRANGEFINDER_ACCURATEWAITER_H
#define LIGHTSPEEDRANGEFINDER_ACCURATEWAITER_H

#include <mbed.h>
#include<TimerEvent.h>
#include <TickerDataClock.h>

/**
 * --------------- AccurateWaiter ---------------
 * ...your order, sir?
 *
 * AccurateWaiter allows threaded applications to have threads wait
 * for a precise amount of time, without sending the CPU into
 * a permanent spinlock.
 *
 * In standard Mbed OS, certain types of threaded applications are limited
 * by the precision of ThisThread::sleep_for(), which only allows sleeping
 * for a whole number of milliseconds.  This limitation comes from
 * the underlying RTX RTOS, which uses a 1 ms scheduling interrupt.
 *
 * What a lot of people don't know is that there is actually a way around
 * this limitation, using the RTOS's EventFlags objects.  Unlike
 * most other RTOS objects, EventFlags are usable from interrupts.  By
 * configuring the Mbed us ticker interrupt to set EventFlags,
 * a waiting thread can be woken immediately after an exact number
 * of ticks on the us ticker.
 *
 * The result is something that combines the best features of wait_us()
 * and ThisThread::sleep_for().  Other threads may run during the wait period
 * (like ThisThread::sleep_for()), but the wait time can be specified to the
 * microsecond (like wait_us()).  The only downside is a bit more overhead
 * when starting the wait operation (to trigger the timer interrupt).
 *
 * I tested this code on my NUCLEO_F429ZI board, and found that it was able
 * to handle any time duration from 1us-1s accurately, with exactly 14us of
 * delay between the set time and the time the wait function returns.  Overhead
 * of calling the function also is around the 15-25us range.
 *
 * Note 1: Since it uses the Mbed us ticker, this class allows waiting for
 * >250,000 years before rollover.
 *
 * Note 2: Each AccurateWaiter object may only be safely used by one thread
 * at a time.
 */
 
class AccurateWaiter : private TimerEvent
{
	// event flags, used to signal thread to wake up from interrupt
	rtos::EventFlags flags;

	// called from timer interrupt
	void handler() override;

public:

	AccurateWaiter();

	/**
	 * Get Mbed's C++ Clock object representing the us ticker.
	 * Use this object to get time_points for wait_until()
	 * @return
	 */
	TickerDataClock & clock()
	{
		return _ticker_data;
	}

	/**
	 * Wait for an exact amount of time.
	 * Other threads will be allowed to run during the wait period.
	 * When the timer expires, the waiting thread will be run immediately, preempting the
	 * current running thread, as long as a few things are true:
	 * - The waiting thread has higher thread priority than the currently running thread
	 * - Interrupts are not disabled
	 *
	 */
	void wait_for(std::chrono::microseconds duration);

	/**
	 * Wait for an exact amount of time.
	 * Overload that casts to us.
	 *
	 * Other threads will be allowed to run during the wait period.
	 * When the timer expires, the waiting thread will be run immediately, preempting the
	 * current running thread, as long as a few things are true:
	 * - The waiting thread has higher thread priority than the currently running thread
	 * - Interrupts are not disabled
	 */
	template<typename _Rep, typename _Period>
	void wait_for(std::chrono::duration<_Rep, _Period> duration)
	{
		wait_for(std::chrono::duration_cast<std::chrono::microseconds>(duration));
	}

	/**
	 * Wait until a specific us timestamp.
	 *
	 * Other threads will be allowed to run during the wait period.
	 * When the timer expires, the waiting thread will be run immediately, preempting the
	 * current running thread, as long as a few things are true:
	 * - The waiting thread has higher thread priority than the currently running thread
	 * - Interrupts are not disabled
	 */
	void wait_until(TickerDataClock::time_point timePoint);
};


AccurateWaiter::AccurateWaiter():
TimerEvent(get_us_ticker_data())
{

}


void AccurateWaiter::handler()
{
	// This signals the RTOS that the waiting thread is ready to wake up.
	flags.set(1);
}

void AccurateWaiter::wait_for(std::chrono::microseconds duration)
{
	// set up timer event to occur
	insert(duration);

	// wait for event flag and then clear it
	flags.wait_all(1);
}

void AccurateWaiter::wait_until(TickerDataClock::time_point timePoint)
{
	// set up timer event to occur
	insert_absolute(timePoint);

	// wait for event flag and then clear it
	flags.wait_all(1);
}

#endif //LIGHTSPEEDRANGEFINDER_ACCURATEWAITER_H

