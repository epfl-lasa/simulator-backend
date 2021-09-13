"""
Imagine you have to run multiple functions within a deadline.
Running them sequentially will be the easiest thing to do, but if one of them behaves badly
and takes too much time, we are in trouble...
This code, instead of running functions sequentially, runs them all in separate threads in parallel.
This allows to consistently call (in time) the functions that meet their deadline, while
ignoring those functions that miss the cycle rate, yet still call them at their maximum rate.
Example:
             plugin1       plugin2         plugin3
start    ----------------------------------------------
              start        start           start

              finish

                           finish
loop rate ----------------------------------------------  (deadline)
              start again  start again
                                           finish
                                           start again
                                        .
                                        .
                                        . etc...
  In this example plugin 1 and 2 meet their deadline (and therefore are "in time"),
  while plugin3 takes more time than the desired call frequency (misses the loop rate), yet
  it is still being called as fast as it can make its computation.
"""

import sys
import time
from threading import Thread


class FuncExecManager:
    """
    Helper class to keep track of synchronous multiple parallel execution of functions with deadlines.
    """

    def __init__(self, list_of_objects, stop_condition, exec_after_each_loop, pause_execution, log_info=print,
                 log_warn=print, log_debug=print):
        # deadline for functions to finish their process
        self._loop_rate = 0.25  # every 4 secs
        # keep track of functions which execution time is "below" the deadline (set by loop rate)
        self._on_time_functions = []
        # keep track of functions which execution time is "above" the deadline (set by loop rate)
        self._late_functions = []
        self._late_threads = []
        # flag to know when a cycle is finished
        self._is_loop_finished = False
        # to differentiate between each cycle (and keep track of functions that finish on time)
        self._cycle_unique_id = None
        # save in member variable the received list of objects
        self._list_of_objects = list_of_objects
        # the function that will be called to check if algorithm should stop or continue
        self._stop_condition = stop_condition
        # this function will be called after each loop
        self._exec_after_each_loop = exec_after_each_loop
        # offer the user the possibility to pause the execution
        self._pause_execution = pause_execution
        # configure loggers
        log_info(f"[FuncExecManager::init] Started synchronous plugin execution manager.")
        self._log_warn = log_warn
        self._log_debug = log_debug

    def _time_control(self, cycle_unique_id, obj):
        class_name = str(obj.__class__)
        # make backup of wall time
        start_time = time.time()
        obj.execute()
        # compare id to check if we are still on time
        if self._cycle_unique_id == cycle_unique_id:
            self._log_debug(f"[FuncExecManager::time_control] Finished {class_name} in time")
            self._on_time_functions.append(obj)
        else:
            end_time = time.time()
            self._log_warn(
                f"[FuncExecManager::time_control] {class_name}: Missed loop rate, took "
                f"{str(round(end_time - start_time - (1.0 / self._loop_rate), 2))} sec longer than expected")
            self._late_functions.append(obj)

    def _loop_thread(self):
        """
        Create an additional thread to monitor the deadline
        """
        time.sleep(1.0 / self._loop_rate)
        self._log_debug('==== loop rate! ====')
        # raise flag to indicate that one loop is complete
        self._is_loop_finished = True

    def start_synchronous_execution(self, loop_rate=0.25):
        # allow user to modify the frequency at which the functions will be called
        self._loop_rate = loop_rate
        # at first all functions are on time
        self._on_time_functions = self._list_of_objects
        self._late_functions = []
        # init cycle_unique_id
        cycle_unique_id = 0
        if cycle_unique_id > sys.maxsize:
            cycle_unique_id = 0
        # run x iterations until user wants to stop (you might want to pass here a ctrl + c detection)
        while self._stop_condition():
            # initialize flag
            self._is_loop_finished = False
            # jump to the next cycle id (because previous loop is finished)
            cycle_unique_id = cycle_unique_id + 1
            self._cycle_unique_id = cycle_unique_id
            # start "on time" functions
            thread_list = []
            for func in self._on_time_functions:
                thread_list.append(Thread(target=self._time_control, args=(cycle_unique_id, func,)))
            self._on_time_functions = []
            for t in thread_list:
                t.start()
            # run loop function on a separate thread
            Thread(target=self._loop_thread).start()
            # wait until loop finishes
            while not self._is_loop_finished:
                # if there is a late thread that finished, run it again right away
                if self._late_functions:
                    self._late_threads = []
                    for func in self._late_functions:
                        self._late_threads.append(Thread(target=self._time_control, args=(cycle_unique_id, func,)))
                        self._late_threads[-1].start()
                    self._late_functions = []
                # sleep to reduce computational load
                time.sleep(0.001)
                # check if user wants to pause execution
                while self._pause_execution():
                    time.sleep(0.1)
            # execute custom function after having finished the loop
            self._exec_after_each_loop()
        # wait for threads to finish
        for thread in self._late_threads:
            thread.join()
