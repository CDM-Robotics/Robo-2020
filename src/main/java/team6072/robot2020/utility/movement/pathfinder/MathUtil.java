/*
 *	  Copyright (C) 2018  John H. Gaby
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, version 3 of the License.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *    
 *    Contact: robotics@gabysoft.com
 */

package team6072.robot2020.utility.movement.pathfinder;

public class MathUtil 
{
	public static double PI = 3.14159265358979323846;
	public static double TAU = PI*2;
	
	public static double bound_radians(double angle) {
	    double newAngle = angle % TAU;
	    if (newAngle < 0) newAngle = TAU + newAngle;
	    return newAngle;
	}
	
	public static double r2d(double angleInRads) {
	    return angleInRads * 180 / PI;
	}
	
	public static double d2r(double angleInDegrees) {
	    return angleInDegrees * PI / 180;
	}
	
	public static double normalizeRadians(double angle)
	{
		if (angle > Math.PI)
		{
			angle -= 2 * Math.PI;
		}
		else if (angle < -Math.PI)
		{
			angle += 2 *Math.PI;
		}
		
		return(angle);
	}
	
	public static double normalizeDegrees(double angle)
	{
		if (angle > 180)
		{
			angle -= 360;
		}
		else if (angle < -180)
		{
			angle += 360;
		}
		
		return(angle);
	}
}
