package pathFinding5;

import java.awt.geom.Point2D;
import java.util.ArrayList;

public class PathData2D
{
	public ArrayList<Point2D.Double> solution = null;
	public double distance = 0.0;
	
	public PathData2D() { return; }
	
	public PathData2D(ArrayList<Point2D.Double> solution, double distance)
	{
		this.solution = solution;
		this.distance = distance;
		return;
	}
}
