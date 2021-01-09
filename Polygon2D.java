package pathFinding5;

import java.awt.Polygon;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;

/**
 * This version has a copy constructor;
 * @author bones
 * @version 2.0 2019-10-31
 */
public class Polygon2D
{
	public double[] xpoints = null;
	public double[] ypoints = null;
	public int npoints = 0;
	public double x = 0.0;
	public double y = 0.0;
	public double width = 0.0;
	public double height = 0.0;
	public double centerx = 0.0;
	public double centery = 0.0;
	
	public Polygon2D() { return; }
	
	public Polygon2D(double[] xpoints, double[] ypoints, int npoints)
	{
		this.xpoints = xpoints;
		this.ypoints = ypoints;
		this.npoints = npoints;
		this.invalidate();
		return;
	}
	
	public Polygon2D(Polygon2D that)
	{
		this.xpoints = new double[that.npoints];
		this.ypoints = new double[that.npoints];
		for(int i = 0; i < that.npoints; i++)
		{
			this.xpoints[i] = that.xpoints[i];
			this.ypoints[i] = that.ypoints[i];
		}
		this.npoints = that.npoints;
		this.invalidate();
		return;
	}
	
	public void invalidate()
	{
		this.x = this.min(this.xpoints);
		this.y = this.min(this.ypoints);
		this.width = this.max(this.xpoints) - this.min(this.xpoints);
		this.height = this.max(this.ypoints) - this.min(this.ypoints);
		this.centerx = this.x + (this.width / 2.0);
		this.centery = this.y + (this.height / 2.0);
		return;
	}
	
	public double getX()
	{
		return this.x;
	}
	
	public double getY()
	{
		return this.y;
	}
	
	public double getWidth()
	{
		return this.width;
	}
	
	public double getHeight()
	{
		return this.height;
	}
	
	public void setCenterX(double x)
	{
		this.centerx = x;
		return;
	}
	
	public double getCenterX()
	{
		return this.centerx;
	}
	
	public void setCenterY(double y)
	{
		this.centery = y;
		return;
	}
	
	public double getCenterY()
	{
		return this.centery;
	}
	
	public void translate(double x, double y)
	{
		for(int i = 0; i < this.npoints; i++)
		{
			this.xpoints[i] += x;
			this.ypoints[i] += y;
		}
		this.invalidate();
		return;
	}
	
	public void moveTo(double x, double y)
	{
		for(int i = 0; i < this.npoints; i++)
		{
			this.xpoints[i] = x + (this.xpoints[i] - this.centerx);
			this.ypoints[i] = y + (this.ypoints[i] - this.centery);
		}
		this.invalidate();
		return;
	}
	
	public void scale(double s)
	{
		for(int i = 0; i < this.npoints; i++)
		{
			this.xpoints[i] = this.xpoints[i] * s;
			this.ypoints[i] = this.ypoints[i] * s;
		}
		return;
	}
	
	private double min(final double[] array)
	{
		double minimum = Float.MAX_VALUE;
		for(double value : array)
		{
			if(value < minimum){
				minimum = value;
			}
		}
		return minimum;
	}
	
	public double minX()
	{
		return this.min(this.xpoints);
	}
	
	public double minY()
	{
		return this.min(this.ypoints);
	}
	
	private double max(final double[] array)
	{
		double maximum = -Float.MAX_VALUE;
		for(double value : array)
		{
			if(value > maximum){
				maximum = value;
			}
		}
		return maximum;
	}
	
	/**
	 * Checks for intersection of an infinite line with the finite line segments of this polygon.
	 * This method assumes this is a closed polygon.
	 * @param x1 int first x value along an infinite line.
	 * @param y1 int first y value along an infinite line.
	 * @param x2 int second x value along an infinite line.
	 * @param y2 int second y value along an infinite line.
	 * @return boolean true in case of intersection, false otherwise.
	 */
	public boolean infiniteLineIntersects(final int x1, final int y1, final int x2, final int y2)
	{
		boolean results = false;
		for(int i = 0; i < this.npoints; i++)
		{
			if(i >= this.npoints - 1){
				results = Line2D.linesIntersect(x1, y1, x2, y2, this.xpoints[i], this.ypoints[i], this.xpoints[0], this.ypoints[0]);
			}else{
				results = Line2D.linesIntersect(x1, y1, x2, y2, this.xpoints[i], this.ypoints[i], this.xpoints[i + 1], this.ypoints[i + 1]);
			}
			if(results){
				break;
			}
		}
		return results;
	}
	
	/**
	 * Checks for intersection of an infinite line with the finite line segments of this polygon.
	 * This method assumes this is a closed polygon.
	 * @param x1 double first x value along an infinite line.
	 * @param y1 double first y value along an infinite line.
	 * @param x2 double second x value along an infinite line.
	 * @param y2 double second y value along an infinite line.
	 * @return boolean true in case of intersection, false otherwise.
	 */
	public boolean infiniteLineIntersects(final double x1, final double y1, final double x2, final double y2)
	{
		boolean results = false;
		for(int i = 0; i < this.npoints; i++)
		{
			if(i >= this.npoints - 1){
				results = Line2D.linesIntersect(x1, y1, x2, y2, this.xpoints[i], this.ypoints[i], this.xpoints[0], this.ypoints[0]);
			}else{
				results = Line2D.linesIntersect(x1, y1, x2, y2, this.xpoints[i], this.ypoints[i], this.xpoints[i + 1], this.ypoints[i + 1]);
			}
			if(results){
				break;
			}
		}
		return results;
	}
	
	/**
	 * Checks for intersection of a finite line segment with the finite line segments of this polygon.
	 * This method assumes this is a closed polygon.
	 * @param x1 int first x value for finite line segment.
	 * @param y1 int first y value for finite line segment.
	 * @param x2 int second x value for finite line segment.
	 * @param y2 int second y value for finite line segment.
	 * @return boolean true in case of intersection, false otherwise.
	 */
	public boolean finiteLineIntersects(final int x1, final int y1, final int x2, final int y2)
	{
		boolean results = false;
		for(int i = 0; i < this.npoints; i++)
		{
			if(i >= this.npoints - 1){
				results = finiteIntersect(x1, y1, x2, y2, this.xpoints[i], this.ypoints[i], this.xpoints[0], this.ypoints[0]);
			}else{
				results = finiteIntersect(x1, y1, x2, y2, this.xpoints[i], this.ypoints[i], this.xpoints[i + 1], this.ypoints[i + 1]);
			}
			if(results){
				break;
			}
		}
		return results;
	}
	
	/**
	 * Checks for intersection of a finite line segment with the finite line segments of this polygon.
	 * This method assumes this is a closed polygon.
	 * @param x1 double first x value for finite line segment.
	 * @param y1 double first y value for finite line segment.
	 * @param x2 double second x value for finite line segment.
	 * @param y2 double second y value for finite line segment.
	 * @return boolean true in case of intersection, false otherwise.
	 */
	public boolean finiteLineIntersects(final double x1, final double y1, final double x2, final double y2)
	{
		boolean results = false;
		for(int i = 0; i < this.npoints; i++)
		{
			if(i >= this.npoints - 1){
				results = finiteIntersect(x1, y1, x2, y2, this.xpoints[i], this.ypoints[i], this.xpoints[0], this.ypoints[0]);
			}else{
				results = finiteIntersect(x1, y1, x2, y2, this.xpoints[i], this.ypoints[i], this.xpoints[i + 1], this.ypoints[i + 1]);
			}
			if(results){
				break;
			}
		}
		return results;
	}
	
	public static boolean finiteIntersect(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4)
 	{
 	  int d1 = computeDirection(x3, y3, x4, y4, x1, y1);
 	  int d2 = computeDirection(x3, y3, x4, y4, x2, y2);
 	  int d3 = computeDirection(x1, y1, x2, y2, x3, y3);
 	  int d4 = computeDirection(x1, y1, x2, y2, x4, y4);
 	  return (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
 	         ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) ||
 	         (d1 == 0 && isOnSegment(x3, y3, x4, y4, x1, y1)) ||
 	         (d2 == 0 && isOnSegment(x3, y3, x4, y4, x2, y2)) ||
 	         (d3 == 0 && isOnSegment(x1, y1, x2, y2, x3, y3)) ||
 	         (d4 == 0 && isOnSegment(x1, y1, x2, y2, x4, y4));
 	}
	
	private static int computeDirection(final double xi, final double yi, final double xj, final double yj, final double xk, final double yk)
 	{
		double a = (xk - xi) * (yj - yi);
		double b = (xj - xi) * (yk - yi);
		return a < b ? -1 : a > b ? 1 : 0;
	}
	
	private static boolean isOnSegment(final double xi, final double yi, final double xj, final double yj, final double xk, final double yk)
 	{
 		return (xi <= xk || xj <= xk) && (xk <= xi || xk <= xj) && (yi <= yk || yj <= yk) && (yk <= yi || yk <= yj);
 	}
	
	// Borrowed from java.awt.Polygon code.
	public boolean contains(final double x, final double y)
	{
		if(this.npoints < 3){
			return false;
		}
		int hits = 0;
		double lastx = this.xpoints[this.npoints - 1];
		double lasty = this.ypoints[this.npoints - 1];
		double curx, cury;
		
		// Walk the edges of the polygon
		for(int i = 0; i < this.npoints; lastx = curx, lasty = cury, i++)
		{
			curx = this.xpoints[i];
			cury = this.ypoints[i];
	
			if(cury == lasty){
				continue;
			}
			
			double leftx;
			if(curx < lastx){
				if(x >= lastx){
					continue;
				}
				leftx = curx;
			}else{
				if(x >= curx){
					continue;
				}
				leftx = lastx;
			}

			double test1, test2;
			if(cury < lasty){
				if(y < cury || y >= lasty){
					continue;
				}
				if(x < leftx){
					hits++;
					continue;
				}
				test1 = x - curx;
				test2 = y - cury;
			}else{
				if(y < lasty || y >= cury){
					continue;
				}
				if(x < leftx){
					hits++;
					continue;
				}
				test1 = x - lastx;
				test2 = y - lasty;
			}

			if(test1 < (test2 / (lasty - cury) * (lastx - curx))){
				hits++;
			}
		}
		return ((hits & 1) != 0);
	}
	
	public boolean contains(final Point2D.Double point)
	{
		return this.contains(point.x, point.y);
	}
	
	public boolean intersects(Polygon2D poly)
	{
		boolean results = false;
		for(int i = 0; i < poly.npoints; i++)
		{
			if(this.contains(poly.xpoints[i], poly.ypoints[i])){
				results = true;
				break;
			}
		}
		return results;
	}
	
	public boolean intersects(double x, double y, double radius)
	{
		boolean intersecting = false;
		for(int i = 0; i < this.npoints; i++)
		{
			if(distance(this.xpoints[i], this.ypoints[i], x, y) < radius){
				intersecting = true;
				break;
			}
		}
		if(!intersecting){
			intersecting = distance(this.centerx, this.centery, x, y) < radius;
		}
		return intersecting;
	}
	
	public Polygon getPolygon()
	{
		int[] xpoints = new int[this.npoints];
		int[] ypoints = new int[this.npoints];
		for(int i = 0; i < this.npoints; i++)
		{
			xpoints[i] = (int)Math.round(this.xpoints[i]);
			ypoints[i] = (int)Math.round(this.ypoints[i]);
		}
		return new Polygon(xpoints, ypoints, this.npoints);
	}
	
	private static double distance(final double x1, final double y1, final double x2, final double y2)
	{
		return Math.sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)));
	}
}
