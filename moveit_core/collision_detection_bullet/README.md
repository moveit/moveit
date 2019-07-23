### Using Bullet as a collision checker [experimental]
To use Bullet as a collision checker you can:
a) create a new planning scene and manually switch to Bullet using the planning_scene.setActiveCollisionDetector()
b) manually switch to Bullet using the `CollisionPluginLoader`
c) manually instantiate a `CollisionRobotBullet` and `CollisionWorldBullet`
d) use rosparam to set the collision checker (preferred but not tested yet) when starting a new `move_group`
