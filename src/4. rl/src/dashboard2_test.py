import time
from multiprocessing import Process,Value
def add_500():
    while True:
        time.sleep(1)
        total.value += 1


def print_val():
    while True:
        time.sleep(0.01)
        print(total.value)


if __name__ == '__main__':
    total = Value('d',-50.111)

    add_process = Process(target=add_500,args=())
    sub_process = Process(target=print_val,args=())

    add_process.start()
    sub_process.start()

    add_process.join()
    sub_process.join()
