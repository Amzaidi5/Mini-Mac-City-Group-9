import math

widthRoad = 90
lengthRoad = 600
tolOffCenter = 5
deltaX_AnchorL = 500
deltaX_AnchorR = 500

Y_car = int(input("Enter the y-coordinate of the car relative to the longitudinal starting point: "))
X_car = int(input("Enter the x-coordinate of the car relative to the latitudinal center: "))

C_distanceAnchorL = math.sqrt((deltaX_AnchorL + widthRoad/2 + X_car)**2 + (lengthRoad - Y_car)**2)
C_distanceAnchorR = math.sqrt((deltaX_AnchorR + widthRoad/2 - X_car)**2 + (lengthRoad - Y_car)**2)
print("Euclidean Distance to Left Anchor:", C_distanceAnchorL)
print("Euclidean Distance to Right Anchor:", C_distanceAnchorR)

# Cosine Law
cosAlphaTag = (C_distanceAnchorL**2 + C_distanceAnchorR**2 - (widthRoad + deltaX_AnchorL + deltaX_AnchorR)**2)/(2 * C_distanceAnchorL * C_distanceAnchorR)

# Trigonometric Identities
sinAlphaTag = math.sqrt(1 - cosAlphaTag**2)

# Sine Law
sinBetaL = C_distanceAnchorR * sinAlphaTag / (widthRoad +  deltaX_AnchorL + deltaX_AnchorR)
betaL = math.asin(sinBetaL)

# Angle Relations
thethaL = math.pi/2 - betaL

# Trigonometric Ratios
dLx = C_distanceAnchorL * math.sin(thethaL)
if dLx < (deltaX_AnchorL + widthRoad/2):
    positionErrorUWB = -(deltaX_AnchorL + widthRoad/2 - dLx)
else:
    positionErrorUWB = dLx - (deltaX_AnchorL + widthRoad/2)

print("Off-Center Distance:", positionErrorUWB)