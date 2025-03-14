import matplotlib.pyplot as plt
import pandas as pd

# تحميل البيانات من الملف
data = pd.read_csv("trajectory.csv")
# رسم المسار
plt.plot(data["y"], data["x"], marker="o", linestyle="-")
plt.xlabel("Y Position")
plt.ylabel("X Position")
plt.title("Robot Trajectory")
plt.grid()
plt.show()
