import math
import os

txt_list = []
with open('val.txt', 'r') as f:
    for line in f:
        line = line.replace("\n", "")
        txt_list.append(line)

txt_list.sort()
print(txt_list)

with open('val_test.txt', 'w') as f:
    for i in range(len(txt_list)):
        content = txt_list[i] + '\n'
        f.write(content)
