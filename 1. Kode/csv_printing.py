import csv

x = ["posx,", "posy"]

with open('Koordinater.csv', 'w', newline='') as csvfile:
    spamwriter = csv.writer(csvfile, delimiter=' ',
                            quotechar='|', quoting=csv.QUOTE_MINIMAL)
    for i in range(10):
        spamwriter.writerow(x)