import multiprocessing as mp

def foo(q):
    q.put('hello')

def funksjon(q2):
    x = 5
    if x < 4:
        test_val = 69
    else:
        test_val = 42
    q2.put(str(test_val))

if __name__ == '__main__':
    mp.set_start_method('spawn')
    q = mp.Queue()
    q2 = mp.Queue()
    p = mp.Process(target=foo, args=(q,))
    p2 = mp.Process(target=funksjon, args=(q2,))
    p.start()
    p2.start()
    queue_value = q.get()
    queue_value2 = q2.get()
    print(queue_value, queue_value2)
    p.join()
    p2.join()
