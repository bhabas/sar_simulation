import os
import time

from multiprocessing import Process, current_process


def square(numbers):
    """ The function squares whatever number it is provided """

    for number in numbers:
        time.sleep(0.5)
        result  = number*number
        print(f"The number {number} squares to result {result}.")


if __name__ == '__main__':

    # The processes list will store each call we make to "square" and the
    # numbers list contatins the numbers we loop through and call the 
    # "square" function on

    numbers = range(100)
    processes = []
    # Loop through the list of numbers, call the "square" function
    for i in range(30):

        process = Process(target=square,args=(numbers,)) # create process
        processes.append(process) # append process to a list

        # Processes are spawned by creating a Process object and
        # then calling its start() method
        process.start()

    for process in processes:
        process.join()
    print("Multiprocessing complete")