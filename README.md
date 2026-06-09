# Evo-UAV

## Overview

Evo is an automated UAV test-generation system built on top of
[Aerialist](https://github.com/skhatiri/Aerialist).
Its goal is to find obstacle configurations that make the PX4
obstacle-avoidance system fail.

Instead of sampling obstacles blindly across the whole map, Evo combines a
**trajectory-aware geometric pre-filter** with a **multi-seed evolutionary
search**:

- **Trajectory-aware pre-filter.** The planned mission trajectory is reconstructed
  from the `.plan` file, and only obstacle configurations whose obstacles fall
  close to the flight path are kept.
- **multi seed.** The budget is split across several independent seeds, each
  optimising its own configuration (optionally anchored to a different segment of
  the trajectory). This keeps the final test suite **diverse**, which is rewarded
  by the competition's diversity score.
- **Evolutionary search.** Each seed starts from a valid configuration and evolves
  it through mutations (position, size, rotation), keeping a child only when its
  fitness improves.

Only the challenging test cases (min distance `< 1.5 m`) are reported, ranked by
danger first and by fewer obstacles second.

## Installation and Usage

### 1. Clone the repository
```bash
git clone https://github.com/mattiadavo/Evo-UAV
cd Evo-UAV/snippets
```

### 2. Create a Docker Image:

```bash
sudo docker build -t [YOUR_IMAGE_NAME] .
```

### 3. Start a container and open a shell
```bash
sudo docker run -dit --name [YOUR_CONTAINER_NAME] [YOUR_IMAGE_NAME]
sudo docker exec -it [YOUR_CONTAINER_NAME] bash
```

### 4. Run the generator
```bash
python3 cli.py generate [PATH_TO_MISSION_YAML] [BUDGET]
```

### 5. Retrieve the results
The generated tests are written inside the container inside
`generated_tests/`
```bash
sudo docker cp [YOUR_CONTAINER_NAME]:/src/generated_tests ./generated_tests
```

## Author

- Mattia Davoli
  - Email: `mattia.dav833@gmail.com`

