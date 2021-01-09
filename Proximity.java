package pathFinding5;

import java.awt.geom.Point2D;

public class Proximity
{
	public static enum Distribution{RADIAL, UNIFORM};
	
	private Distribution mDistribution = Distribution.RADIAL;
	private double mTargetX = 0.0;
	private double mTargetY = 0.0;
	private double mAngle = 0.0; // radians.
	private double mInitialRadius = 0.0;
	private double mInitialIncr = 0.0;
	private double mCurrentRadius = 0.0;
	private double mCurrentIncr = 0.0;
	private int mCurrentStep = 0;
	private int mStepCount = 0;
	private Primary mPrimary = new Primary();
	private CounterClockwise mCounterClockwise = new CounterClockwise();
	private Clockwise mClockwise = new Clockwise();
	private State mCurrent = this.mPrimary;
	
	public Proximity(double subjectX, double subjectY, double targetX, double targetY, double initialRadius, double initialIncrement)
	{
		this.mTargetX = targetX;
		this.mTargetY = targetY;
		this.mCurrentRadius = this.mInitialRadius = initialRadius;
		this.mInitialIncr = initialIncrement;
		this.mCurrentIncr = Math.PI / (initialIncrement);
		this.mAngle = MathUtil.norm(MathUtil.getAngleFromPoints(targetX, targetY, subjectX, subjectY));
		return;
	}
	
	public Proximity(double subjectX, double subjectY, double targetX, double targetY, double initialRadius, double initialIncrement, Distribution distribution)
	{
		this.mTargetX = targetX;
		this.mTargetY = targetY;
		this.mCurrentRadius = this.mInitialRadius = initialRadius;
		this.mInitialIncr = initialIncrement;
		this.mCurrentIncr = Math.PI / (initialIncrement);
		this.mAngle = MathUtil.norm(MathUtil.getAngleFromPoints(targetX, targetY, subjectX, subjectY));
		this.mDistribution = distribution;
		return;
	}
	
	/**
	 * Get the next point from the search algorithm.
	 * @return Point2D.Double.
	 */
	public Point2D.Double get()
	{
		this.mCurrent.update();
		
		/*
		 * After a primary point is found, alternate between clockwise and counterclockwise placement.
		 */
		if(mStepCount == 2){
			mCurrentStep += 1;
			mStepCount = 1;
		}else{
			mStepCount++;
		}
		
		return this.mCurrent.get();
	}
	
	private Point2D.Double findPosition(double angle, double x, double y, double dist)
	{
		double dirX = Math.cos(angle);
		double dirY = -Math.sin(angle);
		return new Point2D.Double(x + (dirX * dist), y + (dirY * dist));
	}
	
	/**
	 * Once a search of points is complete around a circle, expand outward by an increment of the initial radius, and begin 
	 * a new search circle.
	 */
	private void expandSearch()
	{
		this.mCurrentRadius += this.mInitialRadius;
		
		if(this.mDistribution.equals(Distribution.UNIFORM)){
			this.mCurrentIncr = Math.PI / (this.mInitialIncr * ((this.mCurrentRadius / this.mInitialRadius))); // Perfect for even distribution.
		}
		
		this.mCurrentStep = 0;
		this.mCurrent = this.mPrimary;
		return;
	}
	
	private class Primary implements State
	{
		public void update()
		{
			return;
		}
		
		public Point2D.Double get()
		{
			mCurrent = Math.random() < 0.5 ? mCounterClockwise : mClockwise;
			mCurrentStep = 1;
			mStepCount = 0;
			return findPosition(mAngle, mTargetX, mTargetY, mCurrentRadius);
		}
	}
	
	private class CounterClockwise implements State
	{
		public void update()
		{
			if(mCurrentStep * mCurrentIncr >= Math.PI){
				expandSearch();
				return;
			}
			return;
		}
		
		public Point2D.Double get()
		{
			mCurrent = mClockwise;
			return findPosition(MathUtil.norm(mAngle + (mCurrentIncr * mCurrentStep)), mTargetX, mTargetY, mCurrentRadius);
		}
	}
	
	private class Clockwise implements State
	{
		public void update()
		{
			if(mCurrentStep * mCurrentIncr >= Math.PI){
				expandSearch();
				return;
			}
			return;
		}
		
		public Point2D.Double get()
		{
			mCurrent = mCounterClockwise;
			return findPosition(MathUtil.norm(mAngle - (mCurrentIncr * mCurrentStep)), mTargetX, mTargetY, mCurrentRadius);
		}
	}
	
	private interface State
	{
		abstract void update();
		abstract Point2D.Double get();
	}
}
