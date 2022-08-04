import matplotlib.pyplot as plt


# vector for smoothing x3 = [x1, x2, x3, x4, x5] if window = 5
# e.g data = [1,2,3,4,5,6,7,8,9,10]
# x7 smooth = (5+6+7+8+9)/5

# This function takes in a dataset and window value and outputs the smoothed values
xdata = range(11)
ydata = [6, 10, 3, 27, 1, 7, 12, 21, 6, 3, 4]
def smoothData(data, window):

	mvg_avg = []
	if len(data) % window == 0:
		for i in range(len(data)):
			mvg_avg.append(sum(data[i: i + window]) / window)

		return mvg_avg

	else:
		for i in range(len(data)):
			if i <= len(data) - window:
				temp_avg = sum(data[i: i + window]) / window
			
			else:
				temp_avg = data[i]
				
			mvg_avg.append(temp_avg)

		return mvg_avg



smoothed_data = smoothData(ydata, 2)
print(len(smoothed_data))
print(smoothed_data)
print(ydata)

plt.plot(xdata, ydata)
plt.plot(xdata, smoothed_data)

plt.show()
