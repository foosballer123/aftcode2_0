#!/usr/bin/env python3
import time

t_s = time.time()

while (time.time() - t_s) <= 30:

	if (((time.time() - t_s) % 1) == 0):
		print("Hello World!")
