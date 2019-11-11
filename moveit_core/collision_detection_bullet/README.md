### Using Bullet as a collision checker [experimental]
To use Bullet as a collision checker you can manually switch to Bullet using the planning_scene.setActiveCollisionDetector()

### Speed Benchmarks
For speed comparisons to FCL please see (relative link to README of benchmark script will be added).

### Current Limitations
The collision detection using Bullet is not thread safe as the internal collision managers are `mutable` members of the collision environment.
