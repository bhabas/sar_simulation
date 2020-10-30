from multiprocessing import Process, Pipe
import numpy as np


a = np.array([1,2,'t'])

msgs = [a.tolist(),2,3,4,"END"]

def send_msgs(conn,msgs):
    for msg in msgs:
        conn.send(msg)
    conn.close()

def recv_msg(conn):
    while True:
        msg = conn.recv()
        if msg == "END":
            break
        print(msg)

parent_conn, child_conn = Pipe()
p1 = Process(target=send_msgs,args=(parent_conn,msgs))
p2 = Process(target=recv_msg,args=(child_conn,))

p1.start()
p2.start()

p1.join()
p2.join()