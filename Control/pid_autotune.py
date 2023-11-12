import matplotlib
matplotlib.use('Agg')
import PID
import time
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d

P = 1.100
I = 7.374
D = 4.107
pid = PID.PID(P, I, D)
pid.SetPoint = 0.0
pid.setSampleTime(0.01)

total_sampling = 100
feedback = 0
feedback_list = []
time_list = []
setpoint_list = []

# Auto-tuning parameters
tuning_duration = 30  # Duration for auto-tuning in seconds
tuning_output = 1.0  # Output level for auto-tuning
tuning_start_time = time.time()

print("Auto-tuning...")
while time.time() - tuning_start_time < tuning_duration:
    pid.update(feedback)
    output = pid.output

    # Apply auto-tuning output level
    feedback += (output - (1 / (time.time() - tuning_start_time)))

    # Set the setpoint based on the auto-tuning stage
    if 0 < time.time() - tuning_start_time < 10:
        pid.SetPoint = 0.5
    elif 10 <= time.time() - tuning_start_time < 20:
        pid.SetPoint = 1.0
    else:
        pid.SetPoint = 0.0

    time.sleep(0.01)

    feedback_list.append(feedback)
    setpoint_list.append(pid.SetPoint)
    time_list.append(time.time() - tuning_start_time)

# Perform PID parameter calculation based on Ziegler-Nichols method
Ku = max(feedback_list) / tuning_output
Tu = time_list[np.argmax(feedback_list)]
Kp = 0.6 * Ku
Ki = 1.2 * Ku / Tu
Kd = 0.075 * Ku * Tu

# Update PID parameters with auto-tuned values
pid.Kp = Kp
pid.Ki = Ki
pid.Kd = Kd

# Continue simulation with auto-tuned PID
print("Simulating...")
for i in range(total_sampling):
    pid.update(feedback)
    output = pid.output

    if pid.SetPoint > 0:
        feedback += (output - (1 / i))

    if 20 < i < 60:
        pid.SetPoint = 1

    if 60 <= i < 80:
        pid.SetPoint = 0.5

    if i >= 80:
        pid.SetPoint = 1.3

    time.sleep(0.02)

    feedback_list.append(feedback)
    setpoint_list.append(pid.SetPoint)
    time_list.append(i)

time_sm = np.array(time_list)
time_smooth = np.linspace(time_sm.min(), time_sm.max(), 300)

interp_func = interp1d(time_list, feedback_list, kind='cubic')
feedback_smooth = interp_func(time_smooth)

fig1 = plt.gcf()
fig1.subplots_adjust(bottom=0.15)
print(Kp, Ki, Kd)
plt.plot(time_smooth, feedback_smooth, color='red')
plt.plot(time_list, setpoint_list, color='blue')
# plt.xlim((0, total_sampling))
# plt.ylim((min(feedback_list) - 0.5, max(feedback_list) + 0.5))
plt.xlabel('time (s)')
plt.ylabel('PID (PV)')
plt.title('TEST PID')

plt.grid(True)
print("Saving...")
fig1.savefig('result.png', dpi=100)