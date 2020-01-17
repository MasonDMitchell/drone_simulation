#FUNCTION FOUND FROM PROGRAM: 1.981e-08x^4 - 8.304e-06*x^3 +8.272e-04*x^2-9.907e-03*x+5.193e-02

import matplotlib.pyplot as plt
import numpy as np

x = [0,10,20,30,40,50,60,70,80]
y = [.03,.1,.07,.23,.57,.75,.82,1.07,1.11]

plt.plot(x,y)
fit = np.polyfit(x,y,4)
print(fit)
func_y = []
for i in np.arange(0,120,10):
    func_y.append((fit[0]*(i**4))+(fit[1]*(i**3))+(fit[2]*(i**2))+(fit[3]*i)+fit[4])
plt.plot(np.arange(0,120,10),func_y)
plt.show()

