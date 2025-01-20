def filter_data(data_points): 
# Apply median filter to reduce noise 
filtered_points = [] 
for i in range(1, len(data_points) - 1): 
median_distance = median([data_points[i-1][1], data_points[i][1], data_points[i+1][1]]) 
filtered_points.append((data_points[i][0], median_distance)) 
return filtered_points
