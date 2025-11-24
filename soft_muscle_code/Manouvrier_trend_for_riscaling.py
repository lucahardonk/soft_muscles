#the skeletron provided by Ruben was not the right size, thus had to be rescaled.
#we voted for a smaller human size that is 1 meter high, giving us the riscaling to just 48% of the original size of the skeletron, 
#still keeping the same proporton for the rest.
#the omero shoud be aroun 14.66cm in lenght
#56 % will be used as seemd more appropriate lenght!

import pandas as pd
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
import numpy as np

# Your data here
data = {
    "Statura": [140.0, 142.0, 144.0, 145.5, 147.0, 148.8, 149.7, 151.1, 152.8, 154.3, 155.6, 156.8, 158.2, 159.5, 161.2, 163.0, 165.0, 167.0, 169.2, 171.5],
    "Perone": [28.3, 28.8, 29.3, 29.8, 30.3, 30.7, 31.1, 31.6, 32.0, 32.5, 33.0, 33.6, 34.1, 34.6, 35.1, 35.6, 36.1, 36.6, 37.1, 37.6],
    "Tibia": [28.4, 28.9, 29.4, 29.9, 30.4, 30.9, 31.4, 31.9, 32.4, 32.9, 33.4, 34.0, 34.6, 35.2, 35.8, 36.4, 37.0, 37.6, 38.2, 38.8],
    "Femore": [36.3, 36.8, 37.3, 37.8, 38.3, 38.8, 39.3, 39.8, 40.3, 40.8, 41.5, 42.2, 42.9, 43.6, 44.3, 45.0, 45.7, 46.4, 47.1, 47.8],
    "Omero": [26.3, 26.6, 27.0, 27.3, 27.6, 27.9, 28.2, 28.5, 28.9, 29.2, 29.7, 30.2, 30.7, 31.3, 31.8, 32.4, 32.9, 33.4, 33.9, 34.4],
    "Radio": [19.3, 19.5, 19.7, 19.9, 20.1, 20.3, 20.5, 20.7, 20.9, 21.1, 21.4, 21.8, 22.2, 22.6, 23.0, 23.4, 23.8, 24.2, 24.6, 25.0],
    "Ulna": [20.3, 20.6, 20.9, 21.2, 21.5, 21.7, 21.9, 22.2, 22.5, 22.8, 23.1, 23.5, 23.9, 24.3, 24.7, 25.1, 25.5, 25.8, 26.1, 26.4]
}

df = pd.DataFrame(data)

# Plot trends for each column
fig, axes = plt.subplots(3, 2, figsize=(10, 15))
columns = df.columns[1:]  # Exclude 'Statura' from the columns list

for col, ax in zip(columns, axes.flatten()):
    ax.plot(df['Statura'], df[col], marker='o')
    ax.set_title(f'Trend of {col} with Statura')
    ax.set_xlabel('Statura')
    ax.set_ylabel(col)

# Adjust layout
plt.tight_layout()
plt.show()

# Extract the data for 'Statura' and 'Omero'
X = df[['Statura']].values  # Feature matrix
y = df['Omero'].values  # Target vector

# Create and fit the linear regression model
model = LinearRegression()
model.fit(X, y)

# Calculate the slope and intercept
slope = model.coef_[0]
intercept = model.intercept_

# Predict the 'Omero' value for 'Statura' of 1 meter (100 cm)
statura_1m = 100
omero_1m = model.predict([[statura_1m]])

# Plot the original data and the fitted line
plt.figure(figsize=(10, 6))
plt.scatter(X, y, color='orange', label='Original data')
plt.plot(X, model.predict(X), color='blue', linewidth=2, label='Fitted line')
plt.scatter([statura_1m], omero_1m, color='red', zorder=5, label=f'Prediction at 1m: {omero_1m[0]:.2f}')
plt.xlabel('Statura')
plt.ylabel('Omero')
plt.title('Statura vs Omero with Fitted Line')
plt.legend()
plt.grid(True)
plt.show()

slope, intercept, omero_1m[0]

