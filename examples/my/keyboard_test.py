import tkinter as tk
import time
import threading
import numpy as np

class keyboard_control():
	"""docstring for keyboard_control"""
	def __init__(self, control_mode='attitude'):
		self.command = tk.Tk()
		self.command.bind_all("<Key>", self.key_input)

		self.x = 0
		self.y = 0
		self.z = 0
		self.stop = False

	def key_input(self, event):
		key_press = event.keysym.lower()
		print(key_press)

		if key_press == 'w':
			self.y += 1
		elif key_press == 's':
			self.y -= 1
		elif key_press == 'a':
			self.x -= 1
		elif key_press == 'd':
			self.x += 1
		elif key_press == 'return':
			self.z += 1
		elif key_press == 'control_l':
			self.z -= 1
		elif key_press == 'escape':
			self.stop = True

		print("x=%s y=%s z=%s" % (self.x, self.y, self.z))

if __name__ == '__main__':
	remoter = keyboard_control(control_mode='attitude')
	while remoter.stop == False:
		remoter.command.update()
		time.sleep(0.01)