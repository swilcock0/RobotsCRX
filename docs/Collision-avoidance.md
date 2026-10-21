# Collision avoidance

RobotsCRX can check a single-robot cell for collisions and change a program to avoid them. This page covers `Create Program`'s **Collisions** and **Environment** inputs, the `Pick and Place` **Symmetric** input and the `Cell Collisions` component.

## Quick start

1. Model what the robot could hit as meshes: the pedestal, table, walls, fixtures. Connect them to the `Environment` input of `Create Program`.
1. Set `Collisions` to `true`.
1. Read the warnings and errors on the component. A warning says what was changed, and an error is a collision that could not be avoided.

The robot's own base, the tool, any part it holds and the parts picked up by `Set Gripper` are checked automatically. They need no Environment mesh.

## What is checked

- **The tool and any held part** against the robot's base and its arm up to J4.
- **The arm, tool and held part** against the Environment, and against the parts picked up by `Set Gripper`: where they wait before the pick, and where they were placed.
- Shapes closer than 1 mm count as colliding. Parts resting on each other are allowed, as they are shrunk by 2 mm along the gripper's axes at the grasp and release.
- The checks follow the motions the robot takes, including the joint moves between targets, with a pose at least every 10 mm of movement of any point of the robot, tool or held part, and every 10 degrees of the tool's turn. On a joint move, poses far from everything are skipped, as the robot cannot reach anything between them, so a long swing through empty space needs few checks.

Each separate piece of a mesh counts as its convex hull, which is the smallest convex shape around it, like a rubber band stretched around it. Hulls are accurate to about 0.1% of the size, and they are why a cell checks in tens of milliseconds.

> A mesh that is hollow or L-shaped has a hull that fills in the gap, so the robot is kept out of the hollow. Model these as several separate pieces, such as the walls of a shelf as separate boxes, so that each piece is convex.

## What Create Program changes

With `Collisions` on, `Create Program` plans the program before it checks it, changing only these:

1. **Each Pick and Place**, one at a time, trying the options in this order until one has no collisions or errors:
   1. As given.
   1. With the gripper turned 180° about its Z axis, if it clears.
   1. Moving higher between the pick and place, in steps of 100 mm up to 1000 mm, stopping once the lifted points are out of the robot's reach. Each lift tries unturned and then turned.

   The lowest lift that clears is chosen, so neighbouring parts can get different lifts. Options where a straight move changes the arm's configuration (which flips the wrist) are only used if nothing else clears.
1. **Routing**: if no turn or lift clears, the joint moves that collide are routed around the obstacles. Targets are added as joint waypoints, found with RRT-Connect (a randomised search of joint positions), then shortened so the path goes straight between as few waypoints as it can. The same cell always gives the same route, and routes already found are joined first, because later moves usually go around the same obstacles.
1. **Other arm configurations**: if the arm as it is cannot clear the pick and place, the same is tried with the arm reaching the pick and place in its other configurations (shoulder, elbow and wrist up or down), up to 300 mm lifted. The pick and place may use one configuration at the pick and another at the place, as the one that clears the pick may not clear the place. Of the options that clear, the one whose joints move the least is used, so the arm stays as close as it can to how it was.
1. **The rest of the program** gets the same treatment: any joint move that collides is routed around the obstacles, in another configuration of the target if the target itself cannot be reached in the current one. If a collision is left on a linear move, the joint move before it is tried in its other configurations, as that decides the arm for the linear moves after it.

All your other targets are kept exactly as given.

The order is always: keep the arm as it is, then change what it has to. A pick and place that nothing clears is reported, and after two in a row the remaining ones only try the arm as it is, so a setup that cannot work does not make every recompute slow.

### What it cannot change

- **Linear moves** have to stay straight, so a collision on one is an error unless another arm configuration clears it. Add a joint move or a waypoint target before it, or move the obstacle.
- **Configuration you set**: a Pick and Place's template configuration is kept if it clears, and only changed, with a warning, if it does not.
- Only **one robot without external axes** is supported.
- Zones (blending between targets) are checked as the program is simulated, but the controller's own blending is not modelled, so leave some clearance.

## Warnings and errors

| Message | Meaning |
|---|---|
| `Pick and place turned 180° and lifted 300 mm to avoid collisions.` | The targets were made again turned and lifted. Check that the part is symmetric if it was turned (see Symmetric below). |
| `Pick and place routed through 3 joint waypoints to avoid collisions.` | No lift cleared, so the joint moves go around the obstacle through 3 extra joint targets. |
| `Pick and place with the arm in configuration "Elbow" at the pick and "None" at the place to avoid collisions.` | The arm reaches the pick and place in another configuration than the solver chose, which can flip the shoulder, elbow or wrist. |
| `Joint move routed through 2 joint waypoints to avoid collisions.` | A joint move outside a pick and place was routed. With `in configuration "…"`, the target is also reached in that configuration. |
| `Joint move in configuration "Elbow" to avoid collisions.` | A joint move is made to the target in another configuration, so the linear moves after it clear the obstacle. |
| `Pick and place clear of collisions only with a linear motion that changes configuration.` | Every clear option has a straight move that flips the arm. Check the motion on the real robot, or move the pick and place away from the wrist flip boundary, which the `Reachability` component shows. |
| `No turn, lift up to 1000 mm, other configuration or route around obstacles clears this pick and place.` | Nothing found. The collisions are listed as errors. Check that the part and pick and place are reachable and the Environment is not blocking them. |
| `Collision avoidance changed the targets in a way that left errors (…), so they were kept as given.` | The planned targets made a program with errors that the targets as given do not have, such as a target out of reach, so nothing was changed. The collisions are reported as they were. |
| `Collision between robot link 4 and environment mesh 1.` | An error. The first collision on the way to a target, between named shapes: `the robot base`, `robot link N`, `the tool`, `part N (held)`, `part N (waiting)`, `part N (placed)`, `the environment` or `environment mesh N`. |

## Symmetric parts

A part that looks the same when turned 180° about the gripper's Z axis, like a plain strut, can be set `Symmetric` on `Pick and Place`. The program may then pick it one way and place it turned, which gives more options. Leave it off for parts that are not symmetric, as the turned result would show on the finished build.

## Cell Collisions

`Cell Collisions` checks a program that has already been created, and does not change anything. It lists every target with a collision, what collided, and the meshes involved so you can preview them. Its inputs are the Program, the Environment, whether Part Obstacles count, the Clearance, and the linear and angular step sizes. Use it on programs created without `Collisions`, or to see all the collisions rather than the first on the way to each target.

## Tips

- Model the Environment slightly larger than the real thing if clearance matters, as shapes closer than 1 mm count as colliding.
- Keep the picks and places in the robot's clean reach. `Reachability` shows where linear moves flip the wrist.
- Planning checks the program as it is first, so turning `Collisions` on for a program that is already clear costs about one collision check. Only the pick and places that collide are changed, and the variations of each (turns, lifts, configurations) are checked in parallel, so a program of tens of parts takes a few seconds.
- It is quickest when the arm as it is clears. Pick and places that collide where the gripper goes in or out are not lifted, as no lift changes that, the lift a pick and place needed is tried first for the next one, and once a pick and place needed routing, the next ones try routing before lifting.
- From code, `CollisionOptions` (passed to `Program`) also sets the distance, clearance, step sizes, the largest lift and whether parts count.
