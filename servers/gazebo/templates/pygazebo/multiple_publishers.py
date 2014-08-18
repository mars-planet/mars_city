"""
Three tasks running the same factorial coroutine in parallel.
"""
import trollius
from trollius import From


@trollius.coroutine
def factorial(name, number):
    """
    https://en.wikipedia.org/wiki/Factorial
    """
    f = 1
    for i in range(2, number+1):
        print("Task %s: Compute factorial(%s)..." % (name, i))
        yield trollius.sleep(1)
        f *= i
    print("Task %s: factorial(%s) = %s" % (name, number, f))


# Instantiating tasks doesn't cause the coroutine to be run. It merely
# schedules the tasks.
tasks = [
    trollius.Task(factorial("A", 2)),
    trollius.Task(factorial("B", 3)),
    trollius.Task(factorial("C", 4)),
]


# Get the event loop and cause it to run until all the tasks are done.
loop = trollius.get_event_loop()
loop.run_until_complete(trollius.wait(tasks))
loop.close()
