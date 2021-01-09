package pathFinding5;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Collections;

public class PathFinder2D
{
	public static enum Result{RESET, FOUND, NO_SOLUTION, START_TOLERANCE_COLLISION, GOAL_TOLERANCE_COLLISION, GOAL_BY_BERTH_COLLISION};
	public static enum Spur{ISOMETRIC, INCREMENTAL};
	public static enum Enhance{NONE, ALL, START, GOAL, START_AND_GOAL};
	
	private boolean mUseTestMode = false;
	private Avoidable mStart = null;
	private Avoidable mGoal = null;
	private ArrayList<Polygon2D> mTolerances = new ArrayList<Polygon2D>();
	private ArrayList<Point2D.Double> mWayPoints = new ArrayList<Point2D.Double>();
	private ArrayList<Line2D.Double> mTestPaths = null;
	
	private ArrayList<Node> mOpen = new ArrayList<Node>();
	private ArrayList<Node> mClosed = new ArrayList<Node>();
	
	public PathFinder2D() { return;	}
	
	public PathFinder2D(boolean useTestMode)
	{
		this.mUseTestMode = useTestMode;
		this.mTestPaths = new ArrayList<Line2D.Double>();
		return;
	}
	
	public Result search(Avoidable start, Avoidable goal, PathFinder2D.Spur spurType, PathFinder2D.Enhance enhance)
	{
		double subjectBerth = this.mStart.getTolerance().getWidth() / 2.0;
		return this.performSearch(start, goal, spurType, enhance, 1, subjectBerth, 4);
	}
	
	public Result search(Avoidable start, Avoidable goal, PathFinder2D.Spur spurType, PathFinder2D.Enhance enhance, int spurExtensions, double extDist, int numPoints)
	{
		return this.performSearch(start, goal, spurType, enhance, spurExtensions, extDist, numPoints);
	}
	
	// double subjectBerth = this.mStart.getTolerance().getWidth() / 2.0; this works well at extDist.
	private Result performSearch(Avoidable start, Avoidable goal, PathFinder2D.Spur spurType, PathFinder2D.Enhance enhance, int spurExtensions, double extDist, int numPoints)
	{
		this.mStart = start;
		this.mGoal = goal;
		
		if(enhance.equals(Enhance.NONE)){
			// If either the start or goal positions are within a tolerance, there's no solution
			Result preCheck = this.checkForToleranceCollisionsWithStartOrGoal();
			if(preCheck != null){
				return preCheck;
			}
		}
		
		this.mWayPoints.add(new Point2D.Double(this.mStart.getX(), this.mStart.getY()));
		this.mWayPoints.add(new Point2D.Double(this.mGoal.getX(), this.mGoal.getY()));
		this.createTweens();
		int spurExt = enhance.equals(Enhance.ALL) ? spurExtensions + 1 : spurExtensions;
		if(spurType.equals(Spur.ISOMETRIC)){
			this.createIsometricSpurs(this.mStart.getX(), this.mStart.getY(), spurExt, extDist);
			this.createIsometricSpurs(this.mGoal.getX(), this.mGoal.getY(), spurExt, extDist);
		}else if(spurType.equals(Spur.INCREMENTAL)){
			this.createIncrementalSpurs(new Point2D.Double(this.mStart.getX(), this.mStart.getY()), spurExt, extDist, numPoints);
			this.createIncrementalSpurs(new Point2D.Double(this.mGoal.getX(), this.mGoal.getY()), spurExt, extDist, numPoints);
			if(enhance.equals(Enhance.START_AND_GOAL)){
				this.createIncrementalSpur(this.mStart.getTolerance(), new Point2D.Double(this.mGoal.getX(), this.mGoal.getY()), spurExtensions, extDist, numPoints);
				this.createIncrementalSpur(this.mGoal.getTolerance(), new Point2D.Double(this.mGoal.getX(), this.mGoal.getY()), spurExtensions, extDist, numPoints);
			}else if(enhance.equals(Enhance.START)){
				this.createIncrementalSpur(this.mStart.getTolerance(), new Point2D.Double(this.mGoal.getX(), this.mGoal.getY()), spurExtensions, extDist, numPoints);
			}else if(enhance.equals(Enhance.GOAL)){
				//this.createIncrementalSpur(this.mStart.getTolerance(), new Point2D.Double(this.mGoal.getX(), this.mGoal.getY()), spurExtensions, extDist, numPoints);
			}
		}
		
		if(this.mUseTestMode){
			this.mTestPaths = this.drawPaths(this.mWayPoints); // testing feature.
		}
		
		Node startNode = new Node(this.mStart.getX(), this.mStart.getY());
		startNode.parent = null;
		startNode.distanceFromStart = 0.0;
		startNode.distanceToGoal = MathUtil.distance(this.mStart.getX(), this.mStart.getY(), this.mGoal.getX(), this.mGoal.getY());
		startNode.distanceTotal = startNode.distanceFromStart + startNode.distanceToGoal;
		this.mOpen.add(startNode);
		
		Node current = startNode;
		while(!current.equalsPoint(this.mGoal.getX(), this.mGoal.getY()))
		{
			current = this.findRoute();
			if(current == null){
				break;
			}
		}
		
		if(current != null){
			this.drawSolution(this.traceSolution());
			return Result.FOUND;
		}else{
			return Result.NO_SOLUTION;
		}
	}
	
	private Node findRoute()
	{
		Node current = null;
		if(!this.mOpen.isEmpty()){
			
			// Node with shortest total distance should be at the front of the list.
			current = this.mOpen.remove(0);
			this.mClosed.add(current);
			
			if(!current.equalsPoint(this.mGoal.getX(), this.mGoal.getY())){
				this.addChildren(current);
			}
		}
		
		// The open list will be empty if no solution can be found.
		return current;
	}
	
	private void addChildren(Node current)
	{
		double subjectBerth = this.mStart.getTolerance().getWidth() / 2.0;
		for(Point2D.Double p : this.mWayPoints)
		{
			if(!current.equalsPoint(p)){
				boolean isValid = true;
				for(Polygon2D poly : this.mTolerances)
				{
					if(poly.equals(this.mStart.getTolerance())){
						continue;
					}
					if(poly.equals(this.mGoal.getTolerance())){
						continue;
					}
					//The check for finite line intersection needs to also check for distance from tolerances at regular intervals
					//along the line.  Maybe every 5 or 10 pixels.
					if(poly.finiteLineIntersects(current.x, current.y, p.x, p.y)){
						isValid = false;
						break;
					}
					if(!this.isValidByBerth(current.x, current.y, p.x, p.y, 10.0, subjectBerth, this.mTolerances)){
						isValid = false;
						break;
					}
				}
				if(isValid){
					if(this.exists(this.mClosed, p) < 0){
						Node newNode = new Node(p);
						newNode.parent = current;
						newNode.distanceFromStart = MathUtil.distance(newNode.x, newNode.y, this.mStart.getX(), this.mStart.getY());
						newNode.distanceToGoal = MathUtil.distance(newNode.x, newNode.y, this.mGoal.getX(), this.mGoal.getY());
						newNode.distanceTotal = newNode.distanceFromStart + newNode.distanceToGoal;
						if(current.parent != null){
							newNode.angle = getAngle(current, current.parent, newNode);
						}
						
						int openListIndex = this.exists(this.mOpen, newNode);
						if(openListIndex < 0){
							this.mOpen.add(newNode);
						}else{
							//if(newNode.distanceTotal < this.mOpen.get(openListIndex).distanceTotal){
							if(newNode.distanceFromStart < this.mOpen.get(openListIndex).distanceFromStart){
								this.mOpen.get(openListIndex).parent = current;
								this.mOpen.get(openListIndex).distanceFromStart = current.distanceFromStart;
								this.mOpen.get(openListIndex).distanceToGoal = current.distanceToGoal;
								this.mOpen.get(openListIndex).distanceTotal = current.distanceFromStart + current.distanceToGoal;
							}
						}
						
						Collections.sort(this.mOpen);
					}
				}
			}
		}
		return;
	}
	
	//The reason why NO_SOLUTION happens is because the goal is in a tight space where the subject can't fit by berth.
	//There has to be a way to differentiate goal-by-berth and path-by-berth failures.
	//The goal-by-berth failure can be solved by making a pseudo-goal, in proximity to the actual goal, that passes the berth test.
	
	private boolean isValidByBerth(double x1, double y1, double x2, double y2, double incr, double berth, ArrayList<Polygon2D> tolerances)
	{
		boolean result = true;
		double currentX = x1;
		double currentY = y1;
		double direction = MathUtil.getAngleFromPoints(x1, y1, x2, y2);
		double dirX = Math.cos(direction);
		double dirY = -Math.sin(direction);
		while(MathUtil.distance(currentX, currentY, x2, y2) > incr && result == true)
		{
			currentX = currentX + (dirX * incr);
			currentY = currentY + (dirY * incr);
			double testX1 = currentX + ((Math.cos(direction + MathUtil.HALF_PI)) * berth);
			double testY1 = currentY + ((-Math.sin(direction + MathUtil.HALF_PI)) * berth);
			double testX2 = currentX + ((Math.cos(direction - MathUtil.HALF_PI)) * berth);
			double testY2 = currentY + ((-Math.sin(direction - MathUtil.HALF_PI)) * berth);
			for(Polygon2D p : tolerances)
			{
				if(p.contains(testX1, testY1) || p.contains(testX2, testY2)){
					result = false;
					break;
				}
			}
		}
		return result;
	}
	
	/**
	 * @param aList ArrayList<Node> The list to search.
	 * @param target Point2D.Double.
	 * @return int index of ArrayList element matching x and y parameters, or -1 if no match found.
	 */
	private int exists(ArrayList<Node> aList, Point2D.Double target)
	{
		int found = -1;
		int size = aList.size();
		for(int i = 0; i < size; i++)
		{
			if(aList.get(i).x == target.x && aList.get(i).y == target.y){
				found = i;
				break;
			}
		}
		return found;
	}
	
	/**
	 * Trace the solution from the goal back to the start by using each node's parent value. 
	 * @return
	 */
	private ArrayList<Node> traceSolution()
	{
		ArrayList<Node> solution = new ArrayList<Node>();
		if(this.mClosed.size() > 0){
			Node aNode = this.mClosed.get(this.mClosed.size() - 1);
			solution.add(0, aNode);
			while(aNode != null)
			{
				aNode = aNode.parent;
				if(aNode != null){
					solution.add(0, aNode);
				}
			}
		}
		return solution;
	}
	
	public PathData2D getSolution()
	{
		ArrayList<Point2D.Double> points = new ArrayList<Point2D.Double>();
		double distance = 0.0;
		ArrayList<Node> solution = this.traceSolution();
		for(int i = 0; i < solution.size(); i++)
		{
			points.add(solution.get(i));
			if(i < solution.size() - 1){
				distance += MathUtil.distance(solution.get(i).x, solution.get(i).y, solution.get(i + 1).x, solution.get(i).y);
			}
		}
		return new PathData2D(points, distance);
	}
	
	private ArrayList<Line2D.Double> drawSolution(ArrayList<Node> solution)
	{
		ArrayList<Line2D.Double> result = new ArrayList<Line2D.Double>();
		int count = solution.size();
		for(int i = 0; i < count - 1; i++)
		{
			result.add(new Line2D.Double(solution.get(i).x, solution.get(i).y, solution.get(i + 1).x, solution.get(i + 1).y));
		}
		return result;
	}
	
	/**
	 * A "tween" is a point directly between to tolerances.  Each tween represents an opportunity for a path, and
	 * is therefore collected in this.mWaypoints as a Point2D.Double.
	 */
	private void createTweens()
	{
		Polygon2D subject = new Polygon2D(this.mStart.getTolerance());
		int count = this.mTolerances.size();
		for(int i = 0; i < count; i++)
		{
			double ax = this.mTolerances.get(i).x + (this.mTolerances.get(i).width / 2.0);
			double ay = this.mTolerances.get(i).y + (this.mTolerances.get(i).width / 2.0);
			for(int j = i; j < count; j++) // start at i to avoid duplicate comparisons.
			{
				if(i != j){
					double bx = this.mTolerances.get(j).x + (this.mTolerances.get(j).width / 2.0);
					double by = this.mTolerances.get(j).y + (this.mTolerances.get(j).width / 2.0);
					double ax2 = ax - ((ax - bx) / 2.0);
					double ay2 = ay - ((ay - by) / 2.0);
					subject.moveTo(ax2, ay2);
					// If this tween is inside a tolerance, it's a collision, and any path from it would be invalid.
					if(!this.isCollision(subject)){
						this.mWayPoints.add(new Point2D.Double(ax2, ay2));
					}
				}
			}
		}
		return;
	}
	
	private void createIsometricSpurs(double targetX, double targetY, int extensions, double extDist)
	{
		for(Polygon2D poly : this.mTolerances)
		{
			if(poly.equals(this.mStart.getTolerance())){
				continue;
			}
			if(poly.equals(this.mGoal.getTolerance())){
				continue;
			}
			double minDist = Math.sqrt(poly.getWidth() * poly.getWidth() + poly.getHeight() * poly.getHeight()) / 2.0;
			minDist += extDist;
			for(int i = 0; i < extensions; i++)
			{
				double angle = Math.atan2(poly.getCenterY() - targetY, poly.getCenterX() - targetX);
				double length = i == 0 ? minDist : minDist + (i * extDist);
				
				// Rotate it perpendicular to the target.
				angle += (Math.PI / 2.0);
				
				double dirX = Math.cos(angle);
				double dirY = Math.sin(angle);
				double x1 = poly.getCenterX() + (dirX * length);
				double y1 = poly.getCenterY() + (dirY * length);
				double x2 = poly.getCenterX() - (dirX * length);
				double y2 = poly.getCenterY() - (dirY * length);
				
				if(!this.isCollision(x1, y1)){
					this.mWayPoints.add(new Point2D.Double(x1, y1));
				}
				if(!this.isCollision(x2, y2)){
					this.mWayPoints.add(new Point2D.Double(x2, y2));
				}
			}
		}
		return;
	}
	
	private void createIncrementalSpurs(Point2D.Double target, int extensions, double extDist, int numPoints)
	{
		for(Polygon2D tolerance : this.mTolerances)
		{
			if(tolerance.equals(this.mStart.getTolerance())){
				continue;
			}
			if(tolerance.equals(this.mGoal.getTolerance())){
				continue;
			}
			this.createIncrementalSpur(tolerance, target, extensions, extDist, numPoints);
		}
		return;
	}
	
	private void createIncrementalSpur(Polygon2D tolerance, Point2D.Double target, int extensions, double extDist, int numPoints)
	{
		double minDist = Math.sqrt(tolerance.getWidth() * tolerance.getWidth() + tolerance.getHeight() * tolerance.getHeight()) / 2.0;
		minDist += extDist;
		double angle = 0.0; // radians.
		for(int i = 0; i < extensions; i++)
		{
			if(Math.abs(target.x - tolerance.getCenterX()) > 1.0 || Math.abs(target.y - tolerance.getCenterY()) > 1.0){
				angle = Math.atan2(tolerance.getCenterY() - target.y, tolerance.getCenterX() - target.x);
			}
			double length = i == 0 ? minDist : minDist + (i * extDist);
			double incr = MathUtil.PI2 / (double)numPoints;
			for(int j = 0; j < numPoints; j++)
			{
				angle += (incr * j);
				double dirX = Math.cos(angle);
				double dirY = Math.sin(angle);
				double newX = tolerance.getCenterX() + (dirX * length);
				double newY = tolerance.getCenterY() + (dirY * length);
				if(!this.isCollision(newX, newY)){
					this.mWayPoints.add(new Point2D.Double(newX, newY));
				}
			}
		}
		return;
	}
	
	private Result checkForToleranceCollisionsWithStartOrGoal()
	{
		Result result = null;
		for(Polygon2D tolerance : this.mTolerances)
		{
			if(tolerance.equals(this.mStart.getTolerance())){
				continue;
			}
			if(tolerance.equals(this.mGoal.getTolerance())){
				continue;
			}
			if(tolerance.intersects(this.mStart.getTolerance())){
				result = Result.START_TOLERANCE_COLLISION;
				break;
			}else if(tolerance.intersects(this.mGoal.getTolerance())){
				result = Result.GOAL_TOLERANCE_COLLISION;
				break;
			}
		}
		return result;
	}
	
	private boolean isCollision(Point2D.Double p)
	{
		boolean collides = false;
		for(Polygon2D poly : this.mTolerances)
		{
			if(poly.equals(this.mStart.getTolerance())){
				continue;
			}
			if(poly.equals(this.mGoal.getTolerance())){
				continue;
			}
			if(poly.contains(p)){
				collides = true;
				break;
			}
		}
		return collides;
	}
	
	private boolean isCollision(double x, double y)
	{
		boolean collides = false;
		for(Polygon2D poly : this.mTolerances)
		{
			//if(poly.equals(this.mStart.getTolerance())){
			//	continue;
			//}
			if(poly.equals(this.mGoal.getTolerance())){
				continue;
			}
			if(poly.contains(x, y)){
				collides = true;
				break;
			}
		}
		return collides;
	}
	
	private boolean isCollision(Polygon2D p)
	{
		boolean collides = false;
		for(Polygon2D poly : this.mTolerances)
		{
			//if(poly.equals(this.mStart.getTolerance())){
			//	continue;
			//}
			if(poly.equals(this.mGoal.getTolerance())){
				continue;
			}
			if(poly.intersects(p)){
				collides = true;
				break;
			}
		}
		return collides;
	}
	
	public static Polygon2D createRoundTolerance(double x, double y, double radius, int numPoints)
	{
		double increment = MathUtil.PI2 / (double)numPoints;
		double[] xpoints = new double[numPoints];
		double[] ypoints = new double[numPoints];
		double angle = 0.0;
		double dirX = 0.0;
		double dirY = 0.0;
		double newX = 0.0;
		double newY = 0.0;
		for(int i = 0; i < numPoints; i++)
		{
			angle += increment;
			dirX = Math.cos(angle);
			dirY = -Math.sin(angle);
			newX = x + (dirX * radius);
			newY = y + (dirY * radius);
			xpoints[i] = newX;
			ypoints[i] = newY;
		}
		return new Polygon2D(xpoints, ypoints, numPoints);
	}
	
	/**
	 * Computes the angle in radians between three points.
	 * http://stackoverflow.com/questions/1211212/how-to-calculate-an-angle-from-three-points
	 * @param vertex Point2D.Double the vertex or reference point for the angle.
	 * @param p2 Point2D.Double one leg of the angle.
	 * @param p3 Point2D.Double the other leg.
	 * @return double between 0.0 and pi radians.
	 */
	private double getAngle(Point2D.Double vertex, Point2D.Double p2, Point2D.Double p3)
	{
		double a = Math.sqrt((p2.x - vertex.x) * (p2.x - vertex.x) + (p2.y - vertex.y) * (p2.y - vertex.y));
		double b = Math.sqrt((p3.x - vertex.x) * (p3.x - vertex.x) + (p3.y - vertex.y) * (p3.y - vertex.y));
		double c = Math.sqrt((p2.x - p3.x) * (p2.x - p3.x) + (p2.y - p3.y) * (p2.y - p3.y));
		
		return Math.acos((Math.pow(a, 2.0) + Math.pow(b, 2.0) - Math.pow(c, 2.0)) / (2.0 * a * b));
	}
	
	/**
	 * Creates Line2D.Double objects for all suitable paths.
	 * Points are considered unsuitable if they would collide with a tolerance.
	 * This method is intended for testing and demonstration purposes, not for actual applications.
	 * @param wayPoints Vector<Point2D.Double> consisting of tweens, spurs, start and goal points.
	 */
	public ArrayList<Line2D.Double> drawPaths(ArrayList<Point2D.Double> wayPoints)
	{
		ArrayList<Line2D.Double> paths = new ArrayList<Line2D.Double>();
		int count = wayPoints.size();
		for(int i = 0; i < count; i++)
		{
			for(int j = i; j < count; j++) // start at i to avoid duplicate comparisons.
			{
				if(j != i){
					boolean isValid = true;
					for(Polygon2D poly : this.mTolerances)
					{
						if(poly.equals(this.mGoal.getTolerance())){
							continue;
						}
						if(poly.finiteLineIntersects(wayPoints.get(i).x, wayPoints.get(i).y, wayPoints.get(j).x, wayPoints.get(j).y)){
							isValid = false;
							break;
						}
					}
					if(isValid){
						paths.add(new Line2D.Double(wayPoints.get(i), wayPoints.get(j)));
					}
				}
			}
		}
		return paths;
	}
	
	public void addTolerance(Polygon2D item)
	{
		this.mTolerances.add(item);
		return;
	}
	
	public void setTolerances(ArrayList<Polygon2D> tols)
	{
		this.mTolerances = tols;
		return;
	}
	
	public ArrayList<Polygon2D> getTolerances()
	{
		return this.mTolerances;
	}
	
	public ArrayList<Line2D.Double> getTestPaths()
	{
		return this.mTestPaths;
	}
	
	public ArrayList<Point2D.Double> getWayPoints()
	{
		return this.mWayPoints;
	}
	
	public Avoidable getStart()
	{
		return this.mStart;
	}
	
	public Avoidable getGoal()
	{
		return this.mGoal;
	}
	
	/**
	 * A data structure for use with this A* implementation.
	 * @author John McCullock
	 * @version 1.3 2014-07-04
	 */
	@SuppressWarnings("serial")
	public class Node extends Point2D.Double implements Comparable<Node>
	{
		public Node parent = null;
		public double distanceFromStart = 0.0;
		public double distanceToGoal = 0.0;
		public double distanceTotal = 0.0;
		public double angle = -1.0;
		
		public Node(double x, double y)
		{
			this.x = x;
			this.y = y;
			return;
		}
		
		public Node(Point2D.Double location)
		{
			this.x = location.x;
			this.y = location.y;
			return;
		}
		
		public int compareTo(Node that)
		{
			if(this.distanceTotal < that.distanceTotal){
				return -1;
			}else if(this.distanceTotal > that.distanceTotal){
				return 1;
			}else{
				return 0;
			}
		}
		
		public boolean equalsPoint(double x, double y)
		{
			if(this.x == x && this.y == y){
				return true;
			}else{
				return false;
			}
		}
		
		public boolean equalsPoint(Point2D.Double that)
		{
			if(this.x == that.x && this.y == that.y){
				return true;
			}else{
				return false;
			}
		}
	}
}
