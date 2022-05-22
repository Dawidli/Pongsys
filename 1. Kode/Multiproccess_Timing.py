import multiprocessing as mp
import time
global x2
global x
x2 = 0
x = 0

def multi_funksjon(key, q):
    global x
    while True:
        time.sleep(0.5)
        x += 1
        q.put(x)
        print(x)

def multi_funksjon2(key, q2):
    global x2
    while True:
        time.sleep(2)
        x2 += 4
        q2.put(x2)
        print("I CAN ALSO COUNT !!! :D",x2)


if __name__ == '__main__':
    key = 1
    key2 = 2
    q = mp.Queue()
    q2 = mp.Queue()
    p = mp.Process(target=multi_funksjon, args=(key,q))
    p2 = mp.Process(target=multi_funksjon2, args=(key2, q2))
    p2.start()
    p.start()

    p.join()
    p2.join()
