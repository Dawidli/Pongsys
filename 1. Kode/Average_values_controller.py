import numpy as np

# Vi får inn x og y, deretter må e adde nye verdiene til en array av de siste 10 verdiene, og deretter finne pos_avg av de
# Neste steg er å putte pos_avg som input inn i funksjonen
global counter

x = [0]*10 # tom x[]
y = [0]*10 # tom y[]
x_hastighet = [0]*10
y_hastighet = [0]*10

fps = 30
sample_time = 1/fps # fps
counter = 0
grense = len(x)
x_verdier = np.linspace(0,1000,fps*5)
y_verdier = np.linspace(0,500,fps*5)


def find_average(matrise, input):
    global counter
    tot = 0

    for i in range(fps*5):
        counter = counter % grense
        matrise[counter] = round(input[i])
        for I in range(len(matrise)):
            tot += matrise[I]
        avg = tot/len(matrise)
        print(avg)
        counter += 1

def find_hastighet(current, avg, speed_avg):
    global counter

    d = 17.5
    speed = avg/sample_time
    speed[counter] = speed


x_avg = find_average(x, x_verdier)
find_hastighet(x_verdier, x_avg, x_hastighet)

