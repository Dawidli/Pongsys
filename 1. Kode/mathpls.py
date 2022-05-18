import numpy as np

#array = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
#def funksjon():
#    for i in range(20):
#        array[i % 5] += 1
#        print(array)
#    return array
#funksjon()

global fart
global counter
counter = 0
fart = [0]*2
fart1 = 1
fart2 = 2
fart3 = 3
fart4 = 4

def fart_funksjon(ny_fart):
    global counter
    counter = (counter % 2)
    fart[counter] += ny_fart
    counter += 1
    return fart

print(fart)
print(fart_funksjon(fart1))
print(fart_funksjon(fart2))
print(fart_funksjon(fart3))
print(fart_funksjon(fart4))
