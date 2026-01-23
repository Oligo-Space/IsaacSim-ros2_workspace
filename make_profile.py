import pandas as pd
import matplotlib.pyplot as plt
'''
Literally just read timstamp + torque and make a profile
'''
data = pd.read_csv('data_zerosum.csv',usecols=["Time (s)", "Torque X (Nm)", "Torque Y (Nm)", "Torque Z (Nm)"])
wf = open('profile.txt', 'w')
last_t = None
for index, row in data.iterrows():
    t,tx,ty,tz = row
    wf.write(f'T {tx} {ty} {tz}\n')
    if last_t is not None:
        wf.write(f'A {t-last_t}\n')
    last_t = t
wf.close()