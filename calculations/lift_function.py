#FUNCTION FOUND FROM PROGRAM: -.00035x^2 + .03096667x + .04466667

import matplotlib.pyplot as plt
import numpy as np

x = [0,10,20,30,40,50,60,70,80]
y = [.05,.3,.52,.65,.79,.75,.55,.48,.32]

plt.plot(x,y)
fit = np.polyfit(x,y,2)
print(fit)
func_y = []
for i in np.arange(0,120,10):
    func_y.append((fit[0]*(i**2))+(fit[1]*(i**1))+(fit[2]))
plt.plot(np.arange(0,120,10),func_y)
plt.show()

