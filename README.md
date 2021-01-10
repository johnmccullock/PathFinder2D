# PathFinder2D
A variant of the A* algorithm for finding a natural-looking path around obstacles on a 2D map.

A* is a well-worn and very successful pathfinding algorithm, but it requires careful consideration of problem constraints and corner-cases to be of any practical use.  For natural-appearing movements among 2D and 3D objects, their sizes have to be taken into account.

The A* algorithm only deals with points and lines.  Intuitively, most people would think of maps as grids, and their implementation of A* would only consider points neatly arranged along the grid.  However, any realistic-looking movements in an open setting never conform to an arbitrary grid.  Consider ships on the ocean, making smooth, rounded turns, not sharp, angular turns.  And all the ships on the ocean aren't lined up exactly on any grid, their positions and movements just aren't goverened so strictly.

All things considered, a path is what?  A path is an efficent way around obstacles.  When we nagivate around in 2D space, we choose the most efficient waypoints to build a path safely around obstacles.  And it's only those waypoints that are needed for an algorithm like A* to consider.  Our algorithm can ignore the rest of the entire map.

Time to explain some of my terminology (and this project is a testiment to how much I hate coming up with names for things).

(1) Waypoint: A point along a path.
(2) Path: A series of Waypoints leading from a starting point to a goal point.
(3) Avoidable: A discreet area where a path must not intersect.
(4) Tolerance: A polygon surrounding an avoidable, representing the minimum safe distance (berth) from the avoidable.
(5) Spurs: Points around the perimeter of a Tolerance (spurs, like on cowboy boots).
(6) Tweens: Points located half-way between two Tolerances.
(7) Proximal Points: alternative start or goal points that are as close as possible to the start or goal.

The algorithm can arrive at these results:
(1) Found: Best scenario, a nice, complete path from start to goal.
(2) No solution: No possible path was found.
(3) Start collision: The start point coincides with an obstacle's tolerance.
(4) Goal collision: The goal point coincides with an obstacle's tolerance.

My algorithm starts by defining a Tolerance around every Avoidable, sort of like a bounding box.  Each tolerance is either a circle or a polygon, your choice.  The Tolerances are added to a list.

Note that Tolerances can overlap, they're not used for determining collisions like a bounding box, they're only used for considering safe minimum distance from an Avoidable.

Next, Tween points are defined.  For each Tolerance, a Tween point is defined half-way between itself and each other Tolerance.  Tween points that fall within any Tolerance are skipped.  Valid Tweens are added to a list.

Next, Spur points are defined.  There are two ways one can go about this: Incremental and Isometric.  Any number of incremental Spur points can be defined, evenly spaced around the perimiter of a Tolerance.  With isometric Spur points, only two are defined, on either side, perpundicular to the angle between the start point and the Tolerance.  I generally use the incremental method, unless there are so many Tolerances that the resulting number of Spur points slows down the algorithm.  In that case, the isometric method, with it's fewer inherent Spur points, works well enough.  Any Spur points that fall within a Tolerance are skipped.  Again, Tolerances can overlap, so it is easily possible for potential Spur points to fall within a Tolerance.

This next part is something I developed for practical concerns: Start and Goal points can also be Avoidables.  They don't have to be, but you can specify them as such.

This opens up a whole new problem: how can the A* algorithm find a valid path from start to goal if either represent a collision?  My reason for this is to simulate real-world constraints.  For instance, if a ship's goal is a pier, the ship would park along side the pier, not on the pier.  Another example, you park your car along side a house, not on the house.  If the algorithm can't find a valid path to the goal, it should at least try a point close in proximity to the goal.

Which leads to the next part of the algorihtm: defining Proximal points around the start and/or goal points.  A number of Proximal points are defined around the start and/or goal in one of two patterns: Radial or Uniform.  Using the radial method will define Proximal points in concentric circles outward from the start or goal.  If A* can't find a valid path to start or goal, the path can at least start or end close by.  Using the uniform method will make a star-pattern outward from the start or goal.  The uniform method may not be as robust as the radial method, but it uses less points.

Now, A* has all the points it needs to choose a natural-looking path.


The Smoother2D class is a post-processor that smooths out sharp angles for a round, even path. 
