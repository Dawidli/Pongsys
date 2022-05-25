import pandas as pd
import matplotlib.pyplot as plt

plt.rcParams["figure.figsize"] = [4.50, 3.50]
plt.rcParams["figure.autolayout"] = True

headers = ['Name', 'x', 'x_est']

df = pd.read_csv('Koordinater.csv', names=headers)

print(df)
print()
print(df.x[0])

plt.close(1) ;plt.figure(1)
plt.title("This is graf")
plt.plot(df)

plt.show()
