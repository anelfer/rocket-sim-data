# Rocket Simulator Params

_Version 1.8.9_

This project simulates a rocket flight using approximate parameters derived from the SpaceX Falcon 9.

## REST API Endpoints

### 1. Start Simulation
- **Endpoint:** `/simulation/start`
- **Methods:** `POST`
- **Description:** Initiates the rocket simulation. Sending a POST request will start the simulation process.

### 2. Get Simulation Data
- **Endpoint:** `/simulation/data`
- **Method:** `GET`
- **Description:** Retrieves the current simulation data, including flight parameters and rocket status.

### 3. Get Engines Information
- **Endpoint:** `/engines`
- **Method:** `GET`
- **Description:** Provides a list of engines installed on the rocket, along with their current status and operating parameters.

### 4. Update Engine Information
- **Endpoint:** `/engines/{id}`
- **Methods:** `PUT`
- **Description:** Updates the data for a specific engine identified by its `id`. This may include changes to the engine's operating mode, fuel consumption, or other relevant parameters.
