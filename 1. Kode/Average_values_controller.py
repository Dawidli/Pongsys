import numpy as np

# Vi får inn x og y, deretter må e adde nye verdiene til en array av de siste 10 verdiene, og deretter finne pos_avg av de
# Neste steg er å putte pos_avg som input inn i funksjonen
global counter

x = [0]*10 # tom x[]
y = [0]*10 # tom y[]
x_hastighet = [0]*10
y_hastighet = [0]*10

global counter
y = [0]*10 # tom y[]
fps = 30
sample_time = 1/fps # fps
counter = 0
grense = len(x)



def find_average(matrise, input):
    global counter

    tot = 0
    counter = counter % grense
    matrise[counter] = input
    for I in range(len(matrise)):
        tot += matrise[I]
    avg = tot/len(matrise)
    print(avg)
    counter += 1


