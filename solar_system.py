import json




















with open('parameters_solar.json') as f:
    parameters_solar = json.load(f)
print(f"The timestep is {parameters_solar['timestep']}")
for body in parameters_solar['bodies']: print(f"{body['name']} has mass {body['mass']}")