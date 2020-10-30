import os
from multiprocessing import Process, current_process


def square(number):
    result = number*number

    # We can use the "os" module in Python to print out the Process ID
    # assigned to the call of this function assigned by the operating system
    # process_id = os.getpid()
    # print(f"Process ID: {process_id}")

    process_name = current_process().name
    print(f"Process Name: {process_name}")

    # We can also use the "current_process" function to get the name of
    # the Process object.
    print(f"The number {number} squares to result {result}.")



if __name__ == '__main__':
    processes = []
    numbers = [1,2,3,4]

    for number in numbers:
        # Assigning each calc to a process
        process = Process(target=square,args=(number,)) # create process
        processes.append(process) # append process to a list

        # Processes are spawned by creating a Process object and
        # then calling its start() method
        process.start()