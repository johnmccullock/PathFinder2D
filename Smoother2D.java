package pathFinding5;

import java.awt.geom.Point2D;
import java.util.ArrayList;

/**
 * A smoothing filter which attempts to replace sharp angles between points along a path with curved, arcing 
 * segments.
 * @author John McCullock
 * @version 1.0 2014-07-14
 */
public class Smoother2D
{
	private ArrayList<Point2D.Double> mOriginal = new ArrayList<Point2D.Double>();
	private double mIntersectX = 0.0;
	private double mIntersectY = 0.0;
	
	public Smoother2D() { return; }
	
	public Smoother2D(ArrayList<Point2D.Double> path)
	{
		this.mOriginal = path;
		return;
	}
	
	/**
	 * Evaluates all points between first and last along the path, replacing them with arcing paths where possible.
	 * @return ArrayList<Point2D.Double> the modified path.
	 */
	public ArrayList<Point2D.Double> applyFilter()
	{
		if(this.mOriginal.isEmpty()){
			return null;
		}
		int count = this.mOriginal.size();
		ArrayList<Point2D.Double> newPath = new ArrayList<Point2D.Double>();
		newPath.add(this.mOriginal.get(0));
		for(int i = 1; i < count - 1; i++)
		{
			ArrayList<Point2D.Double> temp = null;
			temp = this.locateAnchors(this.mOriginal.get(i).x, this.mOriginal.get(i).y, this.mOriginal.get(i - 1).x, this.mOriginal.get(i - 1).y, this.mOriginal.get(i + 1).x, this.mOriginal.get(i + 1).y);

			if(temp != null){
				for(int j = 0; j < temp.size(); j++)
				{
					newPath.add(temp.get(j));
				}
			}else{
				newPath.add(this.mOriginal.get(i));
			}
		}
		newPath.add(this.mOriginal.get(count - 1));
		
		return newPath;
	}
	
	/**
	 * Takes three points and attempts to an arcing path that could bypass the vertex.
	 * @param ax double x value of center point (vertex).
	 * @param ay double y value of center point (vertex).
	 * @param bx double x value of entry point (where the path is entering).
	 * @param by double y value of entry point (where the path is entering).
	 * @param cx double x value of exit point (where the path would be exiting).
	 * @param cy double y value of exit point (where the path would be exiting).
	 * @return ArrayList<Point2D.Double> containing an arcing path of points.
	 */
	public ArrayList<Point2D.Double> locateAnchors(final double ax, final double ay, final double bx, final double by, final double cx, final double cy)
	{
		double angle1 = this.abs(-Math.atan2(by - ay, bx - ax)); // Entry angle.
		double angle2 = this.abs(-Math.atan2(cy - ay, cx - ax)); // Exit angle.
		double angle1b = this.abs(angle1 + (Math.PI / 2.0));
		double angle2b = this.abs(angle2 + (Math.PI / 2.0));
		double limit = this.getAnchorLimit(ax, ay, bx, by, cx, cy, 100.0);
		
		// Find a point along line a-b where the arc should start.
		double dirX = Math.cos(angle1);
		double dirY = -Math.sin(angle1);
		double point1ax = ax + (dirX * limit);
		double point1ay = ay + (dirY * limit);
		
		// Find a point along line a-c where the arc should end.
		dirX = Math.cos(angle2);
		dirY = -Math.sin(angle2);
		double point2ax = ax + (dirX * limit);
		double point2ay = ay + (dirY * limit);
		
		// Attempt to find a central point which can be used for the center of the arc.
		// Create perpendicular lines from points 1a and 2a, where their intersection would
		// indicate the center.
		
		dirX = Math.cos(angle1b);
		dirY = -Math.sin(angle1b);
		double point1bx = point1ax + (dirX * 200);
		double point1by = point1ay + (dirY * 200);
		
		dirX = Math.cos(angle2b);
		dirY = -Math.sin(angle2b);
		double point2bx = point2ax - (dirX * 200);
		double point2by = point2ay - (dirY * 200);
		
		boolean found = this.lineIntersection(point1bx, point1by, point1ax, point1ay, point2bx, point2by, point2ax, point2ay);
		if(!found){
			return null;
		}
		
		double startAngle = -Math.atan2(point1ay - this.mIntersectY, point1ax - this.mIntersectX);
		double endAngle = -Math.atan2(point2ay - this.mIntersectY, point2ax - this.mIntersectX);
		
		double radius = this.getDistance(point1ax, point1ay, this.mIntersectX, this.mIntersectY);
		ArrayList<Point2D.Double> points = this.createArc(this.mIntersectX, this.mIntersectY, radius, point1ax, point1ay, startAngle, point2ax, point2ay, endAngle);
		
		return points;
	}
	
	/**
	 * Creates ArrayList of points indicating an arcing path.
	 * @param centerX double x value of arc center.
	 * @param centerY double y value of arc center.
	 * @param radius double arc radius.
	 * @param startX double x value of starting point.
	 * @param startY double y value of starting point.
	 * @param startAngle double starting angle in radians found with atan2 (may be negative).
	 * @param endX double x value of ending point.
	 * @param endY double y value of ending point.
	 * @param endAngle double ending angle in radians found with atan2 (may be negative).
	 * @return ArrayList<Point2D.Double> containing points of arcing path.
	 */
	private ArrayList<Point2D.Double> createArc(final double centerX, final double centerY, final double radius, final double startX, final double startY, final double startAngle, final double endX, final double endY, final double endAngle)
	{
		ArrayList<Point2D.Double> points = new ArrayList<Point2D.Double>();
		double increment = Math.PI / (radius / 2.0);
		double start = this.normalize(this.abs(startAngle));
		double end = this.normalize(this.abs(endAngle));
		boolean goLeft = false;
		
		// Negative result means turn to the left, positive means turn right.
		if(this.getCrossProduct(startX, startY, endX, endY, startAngle) < 0.0){
			goLeft = true;
		}
		if(goLeft && end < start){
			end += Math.PI * 2.0;
		}else if(!goLeft && end > start){
			start += Math.PI * 2.0;
		}
		
		if(goLeft){
			double currentAngle = start;
			while(currentAngle < end)
			{
				int x = (int)Math.round(centerX + (int)(Math.cos(currentAngle) * radius));
				int y = (int)Math.round(centerY + (int)(-Math.sin(currentAngle) * radius));
				points.add(new Point2D.Double(x, y));
				currentAngle += increment;
			}
		}else if(!goLeft){
			double currentAngle = start;
			while(currentAngle > end)
			{
				int x = (int)Math.round(centerX + (int)(Math.cos(currentAngle) * radius));
				int y = (int)Math.round(centerY + (int)(-Math.sin(currentAngle) * radius));
				points.add(new Point2D.Double(x, y));
				currentAngle -= increment;
			}
		}
		
		return points;
	}
	
	/**
	 * This helps choose Point1 and Point2 values that will match up more evenly with successive points.
	 * Arcs between neighboring vertexes can extend past each other, creating jagged, erratic paths when 
	 * connecting them.  This function attempts to find a compromise, a middle point between them if 
	 * they're too close to each other.
	 * Both lines from the vertex are compared, and a single value results for both to use.  This has to
	 * be done to unsure an evenly shaped arc.
	 * The maxLimit value acts as an arbitrary length for the arc to keep them from bring generated too large.
	 * @param ax double x value of center point (vertex).
	 * @param ay double y value of center point (vertex).
	 * @param bx double x value of entry point (where the path is entering).
	 * @param by double y value of entry point (where the path is entering).
	 * @param cx double x value of exit point (where the path would be exiting).
	 * @param cy double y value of exit point (where the path would be exiting).
	 * @param maxLimit double maximum length from vertex for the arc.
	 * @return double
	 */
	private double getAnchorLimit(final double ax, final double ay, final double bx, final double by, final double cx, final double cy, final double maxLimit)
	{
		double bDistance = this.getDistance(ax, ay, bx, by);
		double cDistance = this.getDistance(ax, ay, cx, cy);
		double smallest = Math.min(bDistance, cDistance);
		smallest = smallest / 2.0;
		smallest = Math.min(maxLimit, smallest);
		return smallest;
	}
	
	/**
	 * Simply strips the negative sign if one exists.
	 * @param radians double number to evaluate.
	 * @return double the number without its negative sign.
	 */
	private double abs(double radians)
	{
		if(radians == -0.0){
			radians = 0.0;
		}else if(radians == -Math.PI){
			radians = Math.PI;
		}
		return radians;
	}
	
	/**
	 * Simply rolls an angle backward or forward one full turn to get a normal angle between 0.0 and 2PI radians.
	 * @param radians double an angle to evaluate.
	 * @return double normal angle between 0.0 and 2PI radians.
	 */
	private double normalize(double radians)
	{
		if(radians < 0.0){
			radians += Math.PI * 2.0;
		}else if(radians > Math.PI * 2.0){
			radians -= Math.PI * 2.0;
		}
		return radians;
	}
	
	/**
	 * Generates x and y values for point and returns true if intersection is found, or false if no intersection exists.
	 * Found at http://www.ahristov.com/tutorial/geometry-games/intersection-lines.html
	 * @param x1 double x value of first point along first line.
	 * @param y1 double y value of first point along first line.
	 * @param x2 double x value of second point along first line.
	 * @param y2 double y value of second point along first line.
	 * @param x3 double x value of first point along second line.
	 * @param y3 double y value of first point along second line.
	 * @param x4 double x value of second point along second line.
	 * @param y4 double y value of second point along second line.
	 * @return boolean true if intersection is found, false otherwise.
	 */
	private boolean lineIntersection(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4)
	{
		boolean found = false;
		double d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
		if(d != 0.0){
			double xi = ((x3 - x4) * (x1 * y2 - y1 * x2) - (x1 - x2) * (x3 * y4 - y3 * x4)) / d;
			double yi = ((y3 - y4) * (x1 * y2 - y1 * x2) - (y1 - y2) * (x3 * y4 - y3 * x4)) / d;
			this.mIntersectX = xi;
			this.mIntersectY = yi;
			found = true;
		}else{
			found = false;
		}
		return found;
	}
	
	/**
	 * Find which side of a line a point is on.  Useful for finding shortest turning direction, etc.
	 * Found at: http://forum.gpwiki.org/viewtopic.php?t=10128
	 * @param targetX double x value of target point.
	 * @param targetY double y value of target point.
	 * @param currentX double x value of a point on a line.
	 * @param currentY double y value of a point on a line.
	 * @param currentAngle double the angle or direction of the line.
	 * @return double A negative number means the point is on the left, positive means it's on the right.
	 */
	private double getCrossProduct(final double targetX, final double targetY, final double currentX, final double currentY, final double currentAngle)
	{
		double rotX = Math.cos(currentAngle);
		double rotY = -Math.sin(currentAngle);
		double dx = targetX - currentX;
		double dy = targetY - currentY;
		double result = dx * rotY - dy * rotX;
		return result;
	}
	
	/**
	 * Simple squared distance function.
	 * @param ax double x value for point a.
	 * @param ay double y value for point a.
	 * @param bx double x value for point b.
	 * @param by double y value for point b.
	 * @return double distance between points a and b.
	 */
	private double getDistance(final double ax, final double ay, final double bx, final double by)
	{
	    double dX = Math.abs(bx - ax);
	    double dY = Math.abs(by - ay);
	    return Math.sqrt((dX * dX) + (dY * dY));
	}
	
	public void setPath(final ArrayList<Point2D.Double> path)
	{
		this.mOriginal = path;
		return;
	}
}
