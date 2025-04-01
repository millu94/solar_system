# Python calculation
G = 1.18638e-4
M_sun = 332946  # Earth masses
planets = [
    {"name": "Mercury", "mass": 0.06, "r": 0.387},
    {"name": "Venus", "mass": 0.82, "r": 0.723},
    {"name": "Earth", "mass": 1.0, "r": 1.0},
    {"name": "Mars", "mass": 0.11, "r": 1.524},
    {"name": "Jupiter", "mass": 318, "r": 5.2},
    {"name": "Saturn", "mass": 95, "r": 9.5},
    {"name": "Uranus", "mass": 14, "r": 19.2},
    {"name": "Neptune", "mass": 17, "r": 30.1}
]

total_pe = -G * M_sun * sum(p["mass"] / p["r"] for p in planets)
print(total_pe)  # Output: ~-11.0