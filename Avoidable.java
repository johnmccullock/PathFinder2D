package pathFinding5;


public interface Avoidable
{
	abstract double getX();
	abstract double getY();
	abstract double getDirection();
	abstract double getSpeed();
	abstract double getMass();
	abstract Polygon2D getBoundingBox();
	abstract void setTolerance(Polygon2D tolerance);
	abstract Polygon2D getTolerance();
}
