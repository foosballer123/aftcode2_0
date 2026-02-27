#!/usr/bin/env python3
import time

n = 0
t_s = time.time()
while (time.time() - t_s) <= 5:
	time.sleep(1)	
	print("["+str(n)+"] Stalling...")
	n += 1
