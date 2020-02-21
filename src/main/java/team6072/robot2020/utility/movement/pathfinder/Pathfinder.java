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

import team6072.robot2020.utility.movement.pathfinder.PathPoint;
import team6072.robot2020.subsystems.DriveSys;


public class Pathfinder {
	/**
	 * 
	 * @author John Gaby
	 * @author Modified by Christopher Robinson-Perez
	 * 
	 * @brief Specifies a single waypoint for a path
	 * 
	 */
	public static class Waypoint {
		/**
		 * Specifies the horizontal position of the start of this path segment
		 */
		public double x;
		/**
		 * Specifies the vertical position of the start of this path segment
		 */
		public double y;
		/**
		 * Specifies the angle of the start of the path in radians
		 */
		public double angle;
		/**
		 * Specifies the maximum velocity for this portion of the path. If this is set
		 * to zero then the default max velocity is used
		 */
		public double maxVelocity;
		/**
		 * Specifies the distance along tangent for the first Bezier control point P1.
		 * If this value is zero then P1 is calculated based on the length of the path.
		 */
		public double l1 = 0;
		/**
		 * Specifies the distance along tangent for the fourth Bezier control point P4.
		 * If this value is zero then P4 is calculated based on the length of the path.
		 */
		public double l2 = 0;
		/**
		 * Specifies the distance along the tangent for the second Bezier control point
		 * P2. If this value is zero P2 is set equal to P1
		 */
		public double l3 = 0;
		/**
		 * Specifies the distance along tangent for the third Bezier control point P3.
		 * If this value is zero then P3 is set equal to P4.
		 */
		public double l4 = 0;

		/**
		 * This constructor allows you create a waypoint using the starting position,
		 * angle and max velocity. The position of the control points P1, P2, P3, and P4
		 * will be automatically computed based on the length of the path.
		 * 
		 * @param x              - Specifies the starting x position
		 * @param y              - Specifies the starting y Position
		 * @param angle_in       - Specifies the starting angle for the path in radians
		 * @param maxVelocity_in - Specifies the max velocity for the path. If this
		 *                       value is zero then the default max is used.
		 */
		public Waypoint(double x_in, double y_in, double angle_in, double maxVelocity_in) {
			x = x_in;
			y = y_in;
			angle = angle_in;
			maxVelocity = maxVelocity_in;
			l1 = 0;
			l2 = 0;
		}

		/**
		 * This constructor allows you create a waypoint using the starting position and
		 * angle. The max velocity for this path will be set to the default. The
		 * position of the control points P1, P2, P3, and P4 will be automatically
		 * computed based on the length of the path.
		 * 
		 * @param x        - Specifies the starting x position
		 * @param y        - Specifies the starting y Position
		 * @param angle_in - Specifies the starting angle for the path in radians
		 */
		public Waypoint(double x_in, double y_in, double angle_in) {
			x = x_in;
			y = y_in;
			angle = angle_in;
			maxVelocity = 0;
			l1 = 0;
			l2 = 0;
		}

		/**
		 * This constructor allows you create a waypoint using the starting position,
		 * angle, max velocity and positions of the control points P1 and P4. The
		 * control point P2 will be set equal to P1, and the control point P3 will be
		 * set equal to P4.
		 * 
		 * @param x              - Specifies the starting x position
		 * @param y              - Specifies the starting y Position
		 * @param angle_in       - Specifies the starting angle for the path in radians
		 * @param l1_in          - Specifies the distance along the tangent for the
		 *                       first Bezier control point P1
		 * @param l2_in          - Specifies the distance along the tangent for the
		 *                       fourth Bezier control point P4
		 * @param maxVelocity_in - Specifies the max velocity for the path. If this
		 *                       value is zero then the default max is used.
		 */
		public Waypoint(double x_in, double y_in, double angle_in, double l1_in, double l2_in, double maxVelocity_in) {
			x = x_in;
			y = y_in;
			angle = angle_in;
			maxVelocity = maxVelocity_in;
			l1 = l1_in;
			l2 = l2_in;
		}

		/**
		 * This constructor allows you create a waypoint using the starting position,
		 * angle, max velocity and positions of the all the control points P1, P2, P3,
		 * and P4.
		 * 
		 * @param x              - Specifies the starting x position
		 * @param y              - Specifies the starting y Position
		 * @param angle_in       - Specifies the starting angle for the path in radians
		 * @param l1_in          - Specifies the distance along the tangent for the
		 *                       first Bezier control point P1
		 * @param l2_in          - Specifies the distance along the tangent for the
		 *                       fourth Bezier control point P4
		 * @param l3_in          - Specifies the distance along the tangent for the
		 *                       second Bezier control point P2
		 * @param l4_in          - Specifies the distance along the tangent for the
		 *                       third Bezier control point P3
		 * @param maxVelocity_in - Specifies the max velocity for the path. If this
		 *                       value is zero then the default max is used.
		 */
		public Waypoint(double x_in, double y_in, double angle_in, double l1_in, double l2_in, double l3_in,
				double l4_in, double maxVelocity_in) {
			x = x_in;
			y = y_in;
			angle = angle_in;
			maxVelocity = maxVelocity_in;
			l1 = l1_in;
			l2 = l2_in;
			l3 = l3_in;
			l4 = l4_in;
		}

	}

	/**
	 * 
	 * @author John Gaby
	 * 
	 * @brief This class contains the data describing the robot's state at any given
	 *        time and includes it position, velocity, acceleration and heading.
	 * 
	 */
	public static class Segment {
		/**
		 * Specifies the time interval
		 */
		public double dt;
		/**
		 * Specifies the current absolute horizontal position of the robot
		 */
		public double x;
		/**
		 * Specifies the current absolute vertical position of the robot
		 */
		public double y;
		/**
		 * Specifies the distance the robot has moved from the start of the path
		 */
		public double position;
		/**
		 * Specifies the current velocity of the robot
		 */
		public double velocity;
		/**
		 * Specifies the current acceleration of the robot
		 */
		public double acceleration;
		/**
		 * Specifies the current jerk of the robot (i.e. the third derivative of it's
		 * position with respect to time)
		 */
		public double jerk;
		/**
		 * Specifies the current heading in radians
		 */
		public double heading;

		// ! @cond PRIVATE
		public Segment() {
			dt = 0;
			x = 0;
			y = 0;
			position = 0;
			velocity = 0;
			acceleration = 0;
			jerk = 0;
			heading = 0;
		}

		public Segment(Segment seg) {
			dt = seg.dt;
			x = seg.x;
			y = seg.y;
			position = seg.position;
			velocity = seg.velocity;
			acceleration = seg.acceleration;
			jerk = seg.jerk;
			heading = seg.heading;
		}

		public Segment(double dt_in, double x_in, double y_in, double position_in, double acceleration_in,
				double jerk_in, double heading_in) {
			dt = dt_in;
			x = x_in;
			y = y_in;
			position = position_in;
			acceleration = acceleration_in;
			jerk = jerk_in;
			heading = heading_in;
		}
		// ! @endcond
	}

	private static class PathCurve {
		BezierQuintic[] m_bezierPoints;
		PathPoint[] m_center;
		PathPoint[] m_left;
		PathPoint[] m_right;
		Segment[] m_centerSegments;
		Segment[] m_leftSegments;
		Segment[] m_rightSegments;
	}

	private static void computeDistance(PathPoint[] traj, int idx) {
		if (idx > 0) {
			double dx = traj[idx].m_x - traj[idx - 1].m_x;
			double dy = traj[idx].m_y - traj[idx - 1].m_y;
			double d = Math.sqrt(dx * dx + dy * dy);

			traj[idx].m_delta = d;
			traj[idx].m_distance = traj[idx - 1].m_distance + d;
		}
	}

	private static void tankModify(PathPoint[] original, PathPoint[] left_traj, PathPoint[] right_traj, int first,
			int count, double wheelbase_width) {
		double w = wheelbase_width / 2;

		for (int i = 0; i < count; i++) {
			PathPoint seg = original[i + first];

			double cos_angle = Math.cos(seg.m_heading);
			double sin_angle = Math.sin(seg.m_heading);

			left_traj[i + first] = new PathPoint(seg.m_x - (w * sin_angle), seg.m_y + (w * cos_angle), seg.m_heading,
					seg.m_maxVelocity);
			right_traj[i + first] = new PathPoint(seg.m_x + (w * sin_angle), seg.m_y - (w * cos_angle), seg.m_heading,
					seg.m_maxVelocity);

			computeDistance(left_traj, i + first);
			computeDistance(right_traj, i + first);
		}
	}

	@SuppressWarnings("unused")
	private static void printPathPoints(PathPoint[] points) {
		System.out.println("x,y,head,delta,dist");
		for (int j = 0; j < points.length; j++) {
			System.out.println(String.format("%f,%f,%f,%f,%f", points[j].m_x, points[j].m_y,
					MathUtil.normalizeDegrees(MathUtil.r2d(points[j].m_heading)), points[j].m_delta,
					points[j].m_distance));
		}

	}

	private static int findPathPosition(PathPoint[] points, double position, int idx) {
		while ((idx < points.length - 1) && (position > points[idx].m_distance)) {
			idx++;
		}

		return (idx);
	}

	private static int findPathPositionRev(PathPoint[] points, double position, int idx) {
		while ((idx > 0) && (position < points[idx].m_distance)) {
			idx--;
		}

		return (idx);
	}

	private static void addSegment(PathPoint[] points, int idx, int lastIdx, double deltaPos, Segment[] segments,
			int segIdx, double dt, double d, boolean reverse) {
		double x = points[lastIdx].m_x + (points[idx].m_x - points[lastIdx].m_x) * deltaPos;
		double y = points[lastIdx].m_y + (points[idx].m_y - points[lastIdx].m_y) * deltaPos;

		Segment segment = new Segment();

		segment.dt = dt;
		segment.x = x;
		segment.y = y;
		segment.heading = points[idx].m_heading;
		segment.velocity = d / dt;

		double dx;
		double dy;

		if (reverse) {
			dx = (segIdx < segments.length - 1) ? segments[segIdx + 1].x - x : points[points.length - 1].m_x;
			dy = (segIdx < segments.length - 1) ? segments[segIdx + 1].y - y : points[points.length - 1].m_y;
		} else {
			dx = (segIdx > 0) ? x - segments[segIdx - 1].x : x - points[0].m_x;
			dy = (segIdx > 0) ? y - segments[segIdx - 1].y : y - points[0].m_y;
		}
		double angle = Math.atan2(dy, dx);
		double da = MathUtil.normalizeRadians(angle - segment.heading);

		if (Math.abs(da) > Math.PI / 2) {
			segment.velocity = -segment.velocity;
		}

		double dv;

		if (reverse) {
			if (segIdx < segments.length - 1) {
				segment.position = segments[segIdx + 1].position - d;
				dv = segment.velocity - segments[segIdx + 1].velocity;
			} else {
				segment.position = points[points.length - 1].m_distance - d;
				dv = -segment.velocity;
			}
		} else {
			segment.position = (segIdx > 0) ? segments[segIdx - 1].position + d : d;
			dv = (segIdx > 0) ? segment.velocity - segments[segIdx - 1].velocity : segment.velocity;
		}

		segment.acceleration = dv / dt;

		if (reverse) {
			segment.acceleration = -segment.acceleration;
		}

		segments[segIdx] = segment;
	}

	private static int computePath(PathPoint[] left, PathPoint[] center, PathPoint[] right, Segment[] leftSeg,
			Segment[] centerSeg, Segment[] rightSeg, double dt, double max_velocity, double max_acceleration,
			double max_jerk, boolean reverse, double endPosition) {
		int segIdx = reverse ? centerSeg.length - 1 : 0;
		double position = reverse ? center[center.length - 1].m_distance : 0;
		double acceleration = 0;
		double velocity = 0;
		int lastIdx = reverse ? center.length - 1 : 0;

		max_velocity = center[lastIdx].m_maxVelocity;

		double jerkSpeed = max_jerk != 0 ? max_velocity - 0.5 * (max_acceleration * max_acceleration / max_jerk) : 0;

		while (reverse ? (lastIdx > 0) && (velocity < max_velocity)
				: (lastIdx < center.length - 1) && (position < endPosition)) {
			double nextVelocity = velocity;

			if (max_jerk > 0) {
				if (center[lastIdx].m_maxVelocity > max_velocity) {
					max_velocity = center[lastIdx].m_maxVelocity;
					jerkSpeed = max_velocity - 0.5 * (max_acceleration * max_acceleration / max_jerk);
				} else if (center[lastIdx].m_maxVelocity < max_velocity) {
					max_velocity = center[lastIdx].m_maxVelocity;
					jerkSpeed = max_velocity + 0.5 * (max_acceleration * max_acceleration / max_jerk);
				}
			}

			if (jerkSpeed != 0) {
				if (nextVelocity < max_velocity) {
					if (velocity >= jerkSpeed) {
						acceleration -= dt * max_jerk;

						if (acceleration < 0) {
							acceleration = 0;
							nextVelocity = max_velocity;
						} else {
							nextVelocity += dt * acceleration;
						}
					} else {
						acceleration += dt * max_jerk;

						if (acceleration > max_acceleration) {
							acceleration = max_acceleration;
						}

						nextVelocity += dt * acceleration;
					}

					if (nextVelocity > max_velocity) {
						nextVelocity = max_velocity;
					}
				} else if (nextVelocity > max_velocity) {
					if (velocity <= jerkSpeed) {
						acceleration += dt * max_jerk;

						if (acceleration > 0) {
							acceleration = 0;
							nextVelocity = max_velocity;
						} else {
							nextVelocity += dt * acceleration;
						}
					} else {
						acceleration -= dt * max_jerk;

						if (acceleration < -max_acceleration) {
							acceleration = -max_acceleration;
						}

						nextVelocity += dt * acceleration;
					}

					if (nextVelocity < max_velocity) {
						nextVelocity = max_velocity;
					}
				}
			} else {
				if (nextVelocity < max_velocity) {
					nextVelocity += dt * max_acceleration;

					if (nextVelocity > max_velocity) {
						nextVelocity = max_velocity;
					}

				} else if (nextVelocity > max_velocity) {
					nextVelocity -= dt * max_acceleration;

					if (nextVelocity < max_velocity) {
						nextVelocity = max_velocity;
					}
				}
			}

			double d = (dt * (velocity + nextVelocity) / 2);

			int idx;

			if (reverse) {
				idx = findPathPositionRev(center, position - d, lastIdx);
			} else {
				idx = findPathPosition(center, position + d, lastIdx);
			}

			if (idx == lastIdx) {
				if (reverse) {
					lastIdx++;
				} else {
					lastIdx--;
				}
			}

			double D = center[idx].m_distance - center[lastIdx].m_distance;
			double DL = left[idx].m_distance - left[lastIdx].m_distance;
			double DR = right[idx].m_distance - right[lastIdx].m_distance;
			double dl = d * DL / D; // Distance left wheel moved
			double dr = d * DR / D; // Distance right wheel moved

			/*
			 * If the left or right has moved too far, shorten the center to keep it inline
			 */
			if (dl > d) {
				dl = d;
				d = dl * D / DL;
				dr = dl * DR / DL;
			} else if (dr > d) {
				dr = d;
				d = dr * D / DR;
				dl = dr * DL / DR;
			}

			if (reverse) {
				position -= d;
			} else {
				position += d;
			}

			/*
			 * Compute percentage
			 */
			double deltaPos = (position - center[lastIdx].m_distance) / D;

			if ((segIdx < 0) || (segIdx > centerSeg.length)) {
				throw new java.lang.ArrayIndexOutOfBoundsException("Insufficent segement space");
			}

			/*
			 * Create segments
			 */
			addSegment(center, idx, lastIdx, deltaPos, centerSeg, segIdx, dt, d, reverse);
			addSegment(left, idx, lastIdx, deltaPos, leftSeg, segIdx, dt, dl, reverse);
			addSegment(right, idx, lastIdx, deltaPos, rightSeg, segIdx, dt, dr, reverse);

			if (reverse) {
				segIdx--;
			} else {
				segIdx++;
			}
			velocity = nextVelocity;
			lastIdx = idx;

			if (reverse) {
				if (velocity == max_velocity) {
					break;
				}
			}
		}

		// printSegments(centerSeg, leftSeg, rightSeg, segIdx + 1);

		return (segIdx);
	}

	public static void followPath(PathCurve path, double velocity, double dt, double max_velocity,
			double max_acceleration, double max_jerk) {
		// path is the generated path from the waypoints input in drivesys
		// velocity is the currnet velocity of the robot
		// dt is the time interal between points on the curve
		// max_velocit is the maximum velocity the robot can go
		// max_acceleration is exactly what you whoul expect

		/*
		 * Make a rough guess of how many segments we will need
		 */
		double length = path.m_center[path.m_center.length - 1].m_distance;
		int count = (int) (5 * length / (max_velocity * dt));

		path.m_centerSegments = new Segment[count];
		path.m_leftSegments = new Segment[count];
		path.m_rightSegments = new Segment[count];

		/*
		 * First compute the ending deceleration segments
		 */
		Segment[] endCenter = new Segment[count];
		Segment[] endLeft = new Segment[count];
		Segment[] endRight = new Segment[count];

		int endSegIdx = computePath(path.m_left, path.m_center, path.m_right, endLeft, endCenter, endRight, dt,
				max_velocity, max_acceleration, max_jerk, true, 0) + 1;

		/*
		 * Now compute the starting segments up to the start of the deceleration period
		 */
		int segIdx = computePath(path.m_left, path.m_center, path.m_right, path.m_leftSegments, path.m_centerSegments,
				path.m_rightSegments, dt, max_velocity, max_acceleration, max_jerk, false,
				endCenter[endSegIdx].position);

		/*
		 * Now paste together the two sequences
		 */

		for (int s = endSegIdx; s < endCenter.length; s++, segIdx++) {
			path.m_centerSegments[segIdx] = endCenter[s];
			path.m_leftSegments[segIdx] = endLeft[s];
			path.m_rightSegments[segIdx] = endRight[s];
		}

		path.m_centerSegments = java.util.Arrays.copyOf(path.m_centerSegments, segIdx);
		path.m_leftSegments = java.util.Arrays.copyOf(path.m_leftSegments, segIdx);
		path.m_rightSegments = java.util.Arrays.copyOf(path.m_rightSegments, segIdx);

	}

	@SuppressWarnings("unused")
	private static void printSegments(Segment[] center, Segment[] left, Segment[] right, int start) {
		System.out.println("t,dt,cx,cy,cp,dcp,cv,ca,lx,ly,lp,dlp,lv,la,rx,ry,rp,drp,rv,ra");

		double time = 0;

		for (int i = start; (i < center.length); i++) {
			time += center[i].dt;

			double dcp = (i > start) ? center[i].position - center[i - 1].position : 0;
			double dlp = (i > start) ? left[i].position - left[i - 1].position : 0;
			double drp = (i > start) ? right[i].position - right[i - 1].position : 0;

			// System.out.println(String.format("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
			// time, center[i].dt,
			// center[i].x, center[i].y, center[i].position, dcp, center[i].velocity,
			// center[i].acceleration,
			// left[i].x, left[i].y, left[i].position, dlp, left[i].velocity,
			// left[i].acceleration,
			// right[i].x, right[i].y, right[i].position, drp, right[i].velocity,
			// right[i].acceleration));

		}

	}

	/**
	 * 
	 * @author John Gaby
	 * 
	 * @brief This class holds the computed data for a generated path
	 * 
	 */
	public static class Path {
		// ! @cond PRIVATE
		public BezierQuintic[] m_bezierPoints;
		// ! @endcond
		/**
		 * Specifies an array of computed segments defining the motion of the center of
		 * the robot
		 */
		public static Segment[] m_centerPath;
		/**
		 * Specifies an array of computed segments defining the motion of the left
		 * wheels of the robot
		 */
		public Segment[] m_leftPath;
		/**
		 * Specifies an array of computed segments defining the motion of the right
		 * wheels of the robot
		 */
		public Segment[] m_rightPath;

		// ! @cond PRIVATE
		public Path(BezierQuintic[] bezierPoints, Segment[] center, Segment[] left, Segment[] right) {
			m_bezierPoints = bezierPoints;
			m_centerPath = center;
			m_leftPath = left;
			m_rightPath = right;
		}
		// ! @endcond
	}

	@SuppressWarnings("unused")
	private static void printPathPoints(PathPoint[] left, PathPoint[] center, PathPoint[] right) {
		System.out.println("lx,ly,ld,lp,cx,cy,cd,cp,rx,ry,rd,rp,a");

		for (int i = 0; i < center.length; i++) {
			// System.out.println(String.format("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
			// left[i].m_x, left[i].m_y,
			// left[i].m_delta, left[i].m_distance, center[i].m_x, center[i].m_y,
			// center[i].m_delta,
			// center[i].m_distance, right[i].m_x, right[i].m_y, right[i].m_delta,
			// right[i].m_distance,
			// MathUtil.r2d(center[i].m_heading)));
			System.out.println("Heading:" + MathUtil.r2d(center[i].m_heading));

			System.out.println(String.format("%f,%f,%f,%f,%f,%f", left[i].m_delta, left[i].m_distance,
					center[i].m_delta, center[i].m_distance, right[i].m_delta, right[i].m_distance));

		}

	}

	/**
	 * This function accepts an array of waypoints and computes the path of the
	 * robot. The output consists of 3 arrays of data points, one for the center of
	 * the robot, and two others for the left and right wheels. These arrays encode
	 * the robot's (x,y) position, the distance along the curves, the velocity,
	 * acceleration, jerk and heading for each point along the curve separated by
	 * the specified time interval.
	 * 
	 * @param path_in          - Specifies the waypoints from which the path is
	 *                         generated
	 * @param sample_count     - Specifies the number of points to use for each
	 *                         Bezier curver (1000 is a reasonable number)
	 * @param dt               - Specifies the time interval for creating points
	 *                         along the curve
	 * @param max_velocity     - Specifies the default maximum velocity (this value
	 *                         can be overwritten for each portion of the path)
	 * @param max_acceleration - Specifies the maximum acceleration allowed during
	 *                         the acceleration and deceleration phases of the path.
	 * @param max_jerk         - Specifies the maximum jerk allowed during the
	 *                         acceleration and deceleration phases of the path.
	 * @param wheelBase        - Specifies the distance between the wheels of robot.
	 * 
	 */
	public static Path computePath(final Waypoint[] path_in, int sample_count, double dt, double max_velocity,
			double max_acceleration, double max_jerk, double wheelBase) {
		if (path_in.length < 2)
			return (null);

		Waypoint[] path = new Waypoint[path_in.length];

		for (int i = 0; i < path_in.length; i++) {
			path[i] = new Waypoint(path_in[i].x, path_in[i].y, path_in[i].angle, path_in[i].l1, path_in[i].l2,
					path_in[i].l3, path_in[i].l4, path_in[i].maxVelocity > 0 ? path_in[i].maxVelocity : max_velocity);
		}

		PathCurve pathCurve = new PathCurve();

		PathPoint[] centerCurve = new PathPoint[(path.length - 1) * (sample_count)];
		PathPoint[] leftCurve = new PathPoint[centerCurve.length];
		PathPoint[] rightCurve = new PathPoint[centerCurve.length];

		pathCurve.m_center = centerCurve;
		pathCurve.m_left = leftCurve;
		pathCurve.m_right = rightCurve;
		pathCurve.m_bezierPoints = new BezierQuintic[path.length - 1];

		for (int i = 0; i < path.length - 1; i++) {
			BezierQuintic bezier = new BezierQuintic(path[i].x, path[i].y, path[i].angle, path[i].l1, path[i].l3,
					path[i + 1].x, path[i + 1].y, path[i + 1].angle + Math.PI, path[i].l2, path[i].l4);

			bezier.computePathPoints(centerCurve, i * (sample_count), sample_count, path[i].maxVelocity);

			pathCurve.m_bezierPoints[i] = bezier;

			tankModify(centerCurve, leftCurve, rightCurve, i * (sample_count), sample_count, wheelBase);

		}

		System.out.println("Hi there");
		// printPathPoints(leftCurve, centerCurve, rightCurve);
		// System.out.println("Holy shit it did a thing");

		followPath(pathCurve, 0, dt, max_velocity, max_acceleration, max_jerk);

		System.out.println("----------It should have done a thing----------");

		printSegments(pathCurve.m_centerSegments, pathCurve.m_leftSegments, pathCurve.m_rightSegments, 0);

		return (new Path(pathCurve.m_bezierPoints, pathCurve.m_centerSegments, pathCurve.m_leftSegments,
				pathCurve.m_rightSegments));

	}

}
