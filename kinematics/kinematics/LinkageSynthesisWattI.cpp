﻿#include "LinkageSynthesisWattI.h"
#include <opencv2/opencv.hpp>
#include "KinematicUtils.h"
#include "Kinematics.h"
#include "PinJoint.h"
#include "SliderHinge.h"
#include "BoundingBox.h"
#include "LeastSquareSolver.h"
#include "ZOrder.h"
#include "STLExporter.h"
#include "SCADExporter.h"
#include "Vertex.h"
#include "GLUtils.h"

namespace kinematics {

	LinkageSynthesisWattI::LinkageSynthesisWattI(const std::vector<Object25D>& fixed_bodies, const std::vector<std::pair<double, double>>& sigmas, bool avoid_branch_defect, double min_transmission_angle, double min_link_length, const std::vector<double>& weights) {
		this->fixed_bodies = fixed_bodies;
		this->sigmas = sigmas;
		this->avoid_branch_defect = avoid_branch_defect;
		this->min_transmission_angle = min_transmission_angle;
		this->min_link_length = min_link_length;
		this->weights = weights;
	}

	/**
	* Calculate solutions of Watt I.
	*
	* @param poses			three poses
	* @param solutions1	the output solutions for the world coordinates of the driving crank at the first pose, each of which contains a pair of the center point and the circle point
	* @param solutions2	the output solutions for the world coordinates of the follower at the first pose, each of which contains a pair of the center point and the circle point
	*/
	void LinkageSynthesisWattI::calculateSolution(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const std::vector<glm::dvec2>& linkage_avoidance_pts, int num_samples, const std::vector<Object25D>& moving_bodies, std::vector<Solution>& solutions) {
		solutions.clear();

		srand(0);

		int cnt = 0;

		// calculate the bounding boxe of the valid regions
		BBox bbox = boundingBox(linkage_region_pts);

		for (int iter = 0; iter < num_samples && cnt < num_samples; iter++) {
			printf("\rsampling %d/%d", cnt, iter + 1);

			// sample joints within the linkage region
			std::vector<glm::dvec2> points(7);
			for (int i = 0; i < points.size(); i++) {
				while (true) {
					points[i] = glm::dvec2(genRand(bbox.minPt.x, bbox.maxPt.x), genRand(bbox.minPt.y, bbox.maxPt.y));
					if (withinPolygon(linkage_region_pts, points[i])) break;
				}
			}
			
			/*
			// car roof
			points[0] = glm::dvec2(29.847, 8.54938);
			points[1] = glm::dvec2(32.6999, 14.2596);
			points[2] = glm::dvec2(42.8333, 5.67388);
			points[3] = glm::dvec2(33.7698, 9.19524);
			points[4] = glm::dvec2(26.798, 20.1149);
			points[5] = glm::dvec2(42.5028, 8.20261);
			points[6] = glm::dvec2(32.7426, 18.1497);
			*/

			/*
			// folding chair
			points[0] = glm::dvec2(-5.15146, 2.88155);
			points[1] = glm::dvec2(-3.23746, 1.45836);
			points[2] = glm::dvec2(5.78603, 1.05594);
			points[3] = glm::dvec2(-13.2354, 2.98288);
			points[4] = glm::dvec2(5.31989, 10.0897);
			points[5] = glm::dvec2(13.3269, 2.39046);
			points[6] = glm::dvec2(9.94019, 5.93421);
			*/

			if (!optimizeCandidate(poses, linkage_region_pts, bbox, points)) continue;

			// check hard constraints
			std::vector<std::vector<int>> zorder;
			if (!checkHardConstraints(points, poses, linkage_region_pts, linkage_avoidance_pts, moving_bodies, zorder)) continue;
			
			solutions.push_back(Solution(0, points, 0, 0, poses, zorder));
			cnt++;
		}

		printf("\n");
	}

	/**
	* Optimize the linkage parameters based on the rigidity constraints.
	* If it fails to optimize, return false.
	*/
	bool LinkageSynthesisWattI::optimizeCandidate(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const BBox& bbox, std::vector<glm::dvec2>& points) {
		if (!optimizeLink(poses, linkage_region_pts, bbox, points)) return false;
		if (check(poses, points) > 1.0) return false;

		return true;
	}

	/**
	 * @param linkage_region_pts_local	linkage region in the local coordinate system of moving objects (i = 0, 1)
	 * @param bbox_local				bounding box in the local coordinate system of moving objects (i = 0, 1)
	 */
	bool LinkageSynthesisWattI::optimizeLink(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const BBox& bbox, std::vector<glm::dvec2>& points) {
		// setup the initial parameters for optimization
		column_vector starting_point(points.size() * 2);
		for (int i = 0; i < points.size(); i++) {
			starting_point(i * 2, 0) = points[i].x;
			starting_point(i * 2 + 1, 0) = points[i].y;
		}

		try {
			//find_min(dlib::bfgs_search_strategy(), dlib::objective_delta_stop_strategy(1e-7), SolverForWattI(poses), SolverDerivForWattI(poses), starting_point, -1);
			find_min_using_approximate_derivatives(dlib::bfgs_search_strategy(), dlib::objective_delta_stop_strategy(1e-7), SolverForWattI(poses), starting_point, -1);
			
			for (int i = 0; i < points.size(); i++) {
				points[i] = glm::dvec2(starting_point(i * 2, 0), starting_point(i * 2 + 1, 0));
			}
		}
		catch (std::exception& e) {
			return false;
		}

		return true;
	}

	Solution LinkageSynthesisWattI::findBestSolution(const std::vector<std::vector<glm::dmat3x3>>& poses, std::vector<Solution>& solutions, const std::vector<glm::dvec2>& linkage_region_pts, const cv::Mat& dist_map, const BBox& dist_map_bbox, const std::vector<glm::dvec2>& linkage_avoidance_pts, const std::vector<Object25D>& moving_bodies, int num_particles, int num_iterations, bool record_file) {
		// select the best solution based on the trajectory
		if (solutions.size() > 0) {
			particleFilter(solutions, linkage_region_pts, dist_map, dist_map_bbox, linkage_avoidance_pts, moving_bodies, num_particles, num_iterations, record_file);
			return solutions[0];
		}
		else {
			return Solution(0, { { 0, 0 }, { 2, 0 }, { 0, 2 }, { 2, 2 }, { 1, 3 }, { 3, 3 }, { 3, 5 } }, 0, 0, poses);
		}
	}

	double LinkageSynthesisWattI::calculateCost(Solution& solution, const std::vector<Object25D>& moving_bodies, const cv::Mat& dist_map, const BBox& dist_map_bbox) {
		double dist = 0;
		for (int i = 0; i < solution.points.size(); i++) {
			dist += dist_map.at<double>(solution.points[i].y - dist_map_bbox.minPt.y, solution.points[i].x - dist_map_bbox.minPt.x);
		}
		//double tortuosity = tortuosityOfTrajectory(solution.poses, solution.points, moving_body);
		std::vector<glm::dvec2> connected_pts;
		Kinematics kin = constructKinematics(solution.poses, solution.points, solution.zorder, moving_bodies, true, fixed_bodies, connected_pts);
		//double size = glm::length(solution.points[0] - solution.points[2]) + glm::length(solution.points[1] - solution.points[3]) + glm::length(solution.points[0] - connected_pts[0]) + glm::length(solution.points[1] - connected_pts[1]) + glm::length(solution.points[2] - connected_pts[2]) + glm::length(solution.points[3] - connected_pts[3]);
		double size = glm::length(solution.points[0] - solution.points[2]) + glm::length(solution.points[1] - solution.points[3]) + std::max(std::max(glm::length(solution.points[2] - solution.points[3]), glm::length(solution.points[2] - solution.points[5])), glm::length(solution.points[3] - solution.points[5])) + glm::length(solution.points[1] - solution.points[4]) + glm::length(solution.points[3] - solution.points[4]) + glm::length(solution.points[5] - solution.points[6]);

		return solution.position_error * weights[0] + solution.orientation_error * weights[1] + dist * weights[2] + size * weights[4];
	}

	int LinkageSynthesisWattI::getType(const std::vector<glm::dvec2>& points) {
		double g = glm::length(points[0] - points[1]);
		double a = glm::length(points[0] - points[2]);
		double b = glm::length(points[1] - points[3]);
		double h = glm::length(points[2] - points[3]);

		double T1 = g + h - a - b;
		double T2 = b + g - a - h;
		double T3 = b + h - a - g;

		if (T1 < 0 && T2 < 0 && T3 >= 0) {
			return 0;
		}
		else if (T1 >= 0 && T2 >= 0 && T3 >= 0) {
			return 1;
		}
		else if (T1 >= 0 && T2 < 0 && T3 < 0) {
			return 2;
		}
		else if (T1 < 0 && T2 >= 0 && T3 < 0) {
			return 3;
		}
		else if (T1 < 0 && T2 < 0 && T3 < 0) {
			return 4;
		}
		else if (T1 < 0 && T2 >= 0 && T3 >= 0) {
			return 5;
		}
		else if (T1 >= 0 && T2 < 0 && T3 >= 0) {
			return 6;
		}
		else if (T1 >= 0 && T2 >= 0 && T3 < 0) {
			return 7;
		}
		else {
			return -1;
		}
	}

	/**
	* Check if the linkage has a rotatable crank defect.
	* If the crank is not fully rotatable, true is returned.
	*/
	bool LinkageSynthesisWattI::checkRotatableCrankDefect(const std::vector<glm::dvec2>& points) {
		int linkage_type = getType(points);
		int linkage_type2 = getType({ points[3], points[4], points[5], points[6] });

		if ((linkage_type == 0 || linkage_type == 1) && (linkage_type2 == 0 || linkage_type2 == 1)) {
			return false;
		}
		else {
			return true;
		}
	}

	std::pair<double, double> LinkageSynthesisWattI::checkRange(const std::vector<glm::dvec2>& points) {
		double g = glm::length(points[0] - points[1]);
		double a = glm::length(points[0] - points[2]);
		double b = glm::length(points[1] - points[3]);
		double h = glm::length(points[2] - points[3]);

		double T1 = g + h - a - b;
		double T2 = b + g - a - h;
		double T3 = b + h - a - g;

		double theta_min = 0;
		double theta_max = M_PI * 2;

		int linkage_type = getType(points);
		if (linkage_type == 2) {
			if (crossProduct(points[0] - points[2], points[1] - points[0]) >= 0) {
				theta_min = acos((a * a + g * g - (h - b) * (h - b)) / 2 / a / g);
				theta_max = acos((a * a + g * g - (h + b) * (h + b)) / 2 / a / g);
			}
			else {
				theta_min = -acos((a * a + g * g - (h + b) * (h + b)) / 2 / a / g);
				theta_max = -acos((a * a + g * g - (h - b) * (h - b)) / 2 / a / g);
			}
		}
		else if (linkage_type == 3) {
			if (crossProduct(points[0] - points[2], points[1] - points[0]) >= 0) {
				theta_min = acos((a * a + g * g - (b - h) * (b - h)) / 2 / a / g);
				theta_max = acos((a * a + g * g - (b + h) * (b + h)) / 2 / a / g);
			}
			else {
				theta_min = -acos((a * a + g * g - (b + h) * (b + h)) / 2 / a / g);
				theta_max = -acos((a * a + g * g - (b - h) * (b - h)) / 2 / a / g);
			}
		}
		else if (linkage_type == 4 || linkage_type == 7) {
			theta_max = acos((a * a + g * g - (b + h) * (b + h)) / 2 / a / g);
			theta_min = -theta_max;
		}
		else if (linkage_type == 5) {
			theta_min = acos((a * a + g * g - (b - h) * (b - h)) / 2 / a / g);
			theta_max = M_PI * 2 - theta_min;
		}
		else if (linkage_type == 6) {
			theta_min = acos((a * a + g * g - (h - b) * (h - b)) / 2 / a / g);
			theta_max = M_PI * 2 - theta_min;
		}

		return{ theta_min, theta_max };
	}

	bool LinkageSynthesisWattI::checkOrderDefect(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points) {
		glm::dvec2 inv_W = glm::dvec2(glm::inverse(poses[0][0]) * glm::dvec3(points[2], 1));

		int linkage_type = getType(points);
		std::pair<double, double> range = checkRange(points);

		double total_cw = 0;
		double total_ccw = 0;
		double prev = 0;
		for (int i = 0; i < poses[0].size(); i++) {
			// calculate the coordinates of the circle point of the driving crank in the world coordinate system
			glm::dvec2 X = glm::dvec2(poses[0][i] * glm::dvec3(inv_W, 1));
			//std::cout << X.x << "," << X.y << std::endl;

			// calculate the direction from the ground pivot (center point) of the driving crank to the circle point
			glm::dvec2 dir = X - points[0];

			// calculate its angle
			double theta = atan2(dir.y, dir.x);

			if (theta >= prev) {
				if (linkage_type == 0 || linkage_type == 2) {
					total_cw += M_PI * 2 - theta + prev;
					total_ccw += theta - prev;
				}
				else if (linkage_type == 2 || linkage_type == 3 || linkage_type == 4 || linkage_type == 7) {
					total_cw = M_PI * 999; // out of range
					total_ccw += theta - prev;
				}
				else if (linkage_type == 5 || linkage_type == 6) {
					if (theta < range.first) {
						theta += M_PI * 2;
					}
					total_cw = M_PI * 999; // out of range
					total_ccw += theta - prev;
				}
			}
			else {
				if (linkage_type == 0 || linkage_type == 2) {
					total_cw += prev - theta;
					total_ccw += M_PI * 2 - prev + theta;
				}
				else if (linkage_type == 2 || linkage_type == 3 || linkage_type == 4 || linkage_type == 7) {
					total_cw += prev - theta;
					total_ccw = M_PI * 999;	// out of range
				}
				else if (linkage_type == 5 || linkage_type == 6) {
					if (theta < range.first) {
						theta += M_PI * 2;
					}
					total_cw += prev - theta;
					total_ccw = M_PI * 999;	// out of range
				}
			}

			prev = theta;
		}

		if (total_cw > M_PI * 2 && total_ccw > M_PI * 2) return true;
		else return false;
	}

	/**
	 * Check if all the poses are in the same branch.
	 * Drag-link and crank-rocker always do not have a branch defect.
	 * For other types of linkage, the change in the sign of the angle between the coupler and the follower indicates the change of the branch.
	 * If there is an branch defect, true is returned. Otherwise, false is returned.
	 *
	 * @param poses	pose matrices
	 * @param p0		the world coordinates of the fixed point of the driving crank at the first pose
	 * @param p1		the world coordinates of the fixed point of the follower at the first pose
	 * @param p2		the world coordinates of the moving point of the driving crank at the first pose
	 * @param p3		the world coordinates of the moving point of the follower at the first pose
	 * @return		true if the branch defect is detected, false otherwise
	 */
	bool LinkageSynthesisWattI::checkBranchDefect(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points) {
		int type = getType(points);

		// drag-link and crank-rocker always do not have a branch defect
		if (type == 0 || type == 1) return false;

		int orig_sign = 1;

		// calculate the local coordinates of the circle points
		glm::dvec2 p2 = glm::dvec2(glm::inverse(poses[0][0]) * glm::dvec3(points[2], 1));
		glm::dvec2 p3 = glm::dvec2(glm::inverse(poses[0][0]) * glm::dvec3(points[3], 1));

		for (int i = 0; i < poses[0].size(); i++) {
			// calculate the coordinates of the circle point of the driving/driven cranks in the world coordinate system
			glm::dvec2 P2 = glm::dvec2(poses[0][i] * glm::dvec3(p2, 1));
			glm::dvec2 P3 = glm::dvec2(poses[0][i] * glm::dvec3(p3, 1));

			// calculate its sign
			if (i == 0) {
				orig_sign = crossProduct(P3 - P2, points[1] - P3) >= 0 ? 1 : -1;
			}
			else {
				int sign = crossProduct(P3 - P2, points[1] - P3) >= 0 ? 1 : -1;
				if (sign != orig_sign) {
					return true;
				}
			}
		}

		return false;
	}

	bool LinkageSynthesisWattI::checkCircuitDefect(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points) {
		int type = getType(points);

		int orig_sign = 1;
		int orig_sign2 = 1;
		int orig_sign3 = 1;	// sign for the link 1-3-4

		// calculate the local coordinates of the circle points
		glm::dvec2 p2 = glm::dvec2(glm::inverse(poses[0][0]) * glm::dvec3(points[2], 1));
		glm::dvec2 p3 = glm::dvec2(glm::inverse(poses[0][0]) * glm::dvec3(points[3], 1));
		glm::dvec2 p4 = glm::dvec2(glm::inverse(poses[1][0]) * glm::dvec3(points[4], 1));
		glm::dvec2 p5 = glm::dvec2(glm::inverse(poses[0][0]) * glm::dvec3(points[5], 1));
		glm::dvec2 p6 = glm::dvec2(glm::inverse(poses[1][0]) * glm::dvec3(points[6], 1));

		for (int i = 0; i < poses[0].size(); i++) {
			// calculate the coordinates of the circle point of the driving/driven cranks in the world coordinate system
			glm::dvec2 P2 = glm::dvec2(poses[0][i] * glm::dvec3(p2, 1));
			glm::dvec2 P3 = glm::dvec2(poses[0][i] * glm::dvec3(p3, 1));
			glm::dvec2 P4 = glm::dvec2(poses[1][i] * glm::dvec3(p4, 1));
			glm::dvec2 P5 = glm::dvec2(poses[0][i] * glm::dvec3(p5, 1));
			glm::dvec2 P6 = glm::dvec2(poses[1][i] * glm::dvec3(p6, 1));

			// calculate its sign
			if (i == 0) {
				if (type == 0) {
					orig_sign = crossProduct(P2 - points[0], P3 - P2) >= 0 ? 1 : -1;
				}
				else if (type == 1) {
					orig_sign = crossProduct(P3 - P2, points[1] - P3) >= 0 ? 1 : -1;
				}
				else if (type == 2) {
					orig_sign = crossProduct(points[0] - points[1], P2 - points[0]) >= 0 ? 1 : -1;
				}
				else if (type == 3) {
					orig_sign = crossProduct(points[1] - P3, points[0] - points[1]) >= 0 ? 1 : -1;
				}

				orig_sign2 = crossProduct(P6 - P5, P4 - P6) >= 0 ? 1 : -1;
				orig_sign3 = crossProduct(P4 - points[1], P3 - P4) >= 0 ? 1 : -1;
			}
			else {
				int sign, sign2, sign3;
				if (type == 0) {
					sign = crossProduct(P2 - points[0], P3 - P2) >= 0 ? 1 : -1;
				}
				else if (type == 1) {
					sign = crossProduct(P3 - P2, points[1] - P3) >= 0 ? 1 : -1;
				}
				else if (type == 2) {
					sign = crossProduct(points[0] - points[1], P2 - points[0]) >= 0 ? 1 : -1;
				}
				else if (type == 3) {
					sign = crossProduct(points[1] - P3, points[0] - points[1]) >= 0 ? 1 : -1;
				}
				else {
					sign = orig_sign;
				}

				sign2 = crossProduct(P6 - P5, P4 - P6) >= 0 ? 1 : -1;
				sign3 = crossProduct(P4 - points[1], P3 - P4) >= 0 ? 1 : -1;

				if (sign != orig_sign || sign2 != orig_sign2 || sign3 != orig_sign3) return true;
			}
		}

		return false;
	}

	/**
	* Construct a linkage.
	*/
	Kinematics LinkageSynthesisWattI::constructKinematics(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points, const std::vector<std::vector<int>>& zorder, const std::vector<Object25D>& moving_bodies, bool connect_joints, const std::vector<Object25D>& fixed_bodies, std::vector<glm::dvec2>& connected_pts) {
		Kinematics kin;
		kin.linkage_type = 0;
		kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(0, true, points[0], zorder.size() == 3 ? zorder[2][0] : 1)));
		kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(1, true, points[1], zorder.size() == 3 ? zorder[2][1] : 1)));
		kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(2, false, points[2], zorder.size() == 3 ? std::max(zorder[2][0], zorder[2][2]) : 1)));
		kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(3, false, points[3], zorder.size() == 3 ? zorder[2][2] : 1)));
		kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(4, false, points[4], zorder.size() == 3 ? zorder[2][1] : 1)));
		kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(5, false, points[5], zorder.size() == 3 ? std::max(zorder[2][2], zorder[2][3]) : 1)));
		kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(6, false, points[6], zorder.size() == 3 ? zorder[2][3] : 1)));
		kin.diagram.addLink(true, kin.diagram.joints[0], kin.diagram.joints[2], true, zorder.size() == 3 ? zorder[2][0] : 1);
		kin.diagram.addLink(false, { kin.diagram.joints[1], kin.diagram.joints[3], kin.diagram.joints[4] }, true, zorder.size() == 3 ? zorder[2][1] : 1);
		kin.diagram.addLink(false, { kin.diagram.joints[2], kin.diagram.joints[3], kin.diagram.joints[5] }, true, zorder.size() == 3 ? zorder[2][2] : 1);
		kin.diagram.addLink(false, kin.diagram.joints[4], kin.diagram.joints[6], false);
		kin.diagram.addLink(false, kin.diagram.joints[5], kin.diagram.joints[6], true, zorder.size() == 3 ? zorder[2][3] : 1);

		std::vector<Object25D> copied_fixed_bodies = fixed_bodies;

		// update the geometry
		updateMovingBodies(kin, moving_bodies);

		if (connect_joints) {
			kin.diagram.connectJointsToBodies(copied_fixed_bodies, zorder, connected_pts);
		}

		// add the fixed rigid bodies
		for (int i = 0; i < copied_fixed_bodies.size(); i++) {
			kin.diagram.addBody(kin.diagram.joints[0], kin.diagram.joints[1], copied_fixed_bodies[i]);
		}

		if (zorder.size() == 3 && kin.diagram.connectors.size() == 10) {
			kin.diagram.connectors[0].z = zorder[0][0];
			kin.diagram.connectors[1].z = zorder[0][1];
			kin.diagram.connectors[2].z = zorder[1][0];
			kin.diagram.connectors[3].z = zorder[1][1];
			kin.diagram.connectors[4].z = zorder[1][2];
			kin.diagram.connectors[5].z = zorder[1][3];
			kin.diagram.connectors[6].z = zorder[2][0];
			kin.diagram.connectors[7].z = zorder[2][1];
			kin.diagram.connectors[8].z = zorder[2][2];
			kin.diagram.connectors[9].z = zorder[2][3];
		}

		// calculte the range of motion
		/*
		std::pair<double, double> angle_range = checkRange(poses, points);
		kin.min_angle = angle_range.first;
		kin.max_angle = angle_range.second;
		*/

		return kin;
	}

	/**
	* Construct a linkage.
	*/
	void LinkageSynthesisWattI::updateMovingBodies(Kinematics& kin, const std::vector<Object25D>& moving_bodies) {
		kin.diagram.bodies.clear();
		kin.diagram.addBody(kin.diagram.joints[2], kin.diagram.joints[5], moving_bodies[0]);
		kin.diagram.addBody(kin.diagram.joints[4], kin.diagram.joints[6], moving_bodies[1]);
	}

	bool LinkageSynthesisWattI::checkHardConstraints(std::vector<glm::dvec2>& points, const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const std::vector<glm::dvec2>& linkage_avoidance_pts, const std::vector<Object25D>& moving_bodies, std::vector<std::vector<int>>& zorder) {
		if (glm::length(points[0] - points[1]) < min_link_length) return false;
		if (glm::length(points[2] - points[3]) < min_link_length) return false;

		//if (checkFolding(points)) continue;
		if (avoid_branch_defect && checkBranchDefect(poses, points)) return false;
		if (checkCircuitDefect(poses, points)) return false;
		//if (checkOrderDefect(poses, points)) return false;

		// collision check
		if (checkCollision(poses, points, fixed_bodies, moving_bodies)) return false;

		// record collision between connectors
		Kinematics kin = recordCollisionForConnectors(poses, points, fixed_bodies, moving_bodies);

		// determine the z-order of links and connectors
		try {
			zorder = ZOrder::zorderConnectors(kin.diagram.connectors);
		}
		catch (char* ex) {
			return false;
		}

		return true;
	}

	bool LinkageSynthesisWattI::checkCollision(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points, const std::vector<Object25D>& fixed_bodies, const std::vector<Object25D>& moving_bodies) {
		std::vector<glm::dvec2> connector_pts;
		kinematics::Kinematics kinematics = constructKinematics(poses, points, {}, moving_bodies, false, fixed_bodies, connector_pts);
		kinematics.diagram.initialize();

		// calculate the rotational angle of the driving crank for 1st, 2nd, and last poses
		// i.e., angles[0] = first pose, angles[1] = second pose, angles[2] = last pose
		std::vector<double> angles(3);
		glm::dvec2 w(glm::inverse(poses[0][0]) * glm::dvec3(points[2], 1));
		for (int i = 0; i < 2; i++) {
			glm::dvec2 W = glm::dvec2(poses[0][i] * glm::dvec3(w, 1));
			angles[i] = atan2(W.y - points[0].y, W.x - points[0].x);
		}
		{
			glm::dvec2 W = glm::dvec2(poses[0].back() * glm::dvec3(w, 1));
			angles[2] = atan2(W.y - points[0].y, W.x - points[0].x);
		}

		// order the angles based on their signs
		int type = 0;
		if (angles[0] < 0 && angles[1] < 0 && angles[2] >= 0 && angles[0] >= angles[1]) {
			type = 1;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] < 0 && angles[0] < angles[2]) {
			type = 2;
			angles[1] -= M_PI * 2;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] < 0 && angles[0] >= angles[2]) {
			type = 3;
			angles[2] += M_PI * 2;
		}
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] >= 0 && (poses[0].size() >= 3 && angles[1] >= angles[2] || poses[0].size() == 2 && angles[1] - angles[0] > M_PI)) {
			type = 4;
			angles[1] -= M_PI * 2;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] < 0 && (poses[0].size() >= 3 && angles[1] < angles[2] || poses[0].size() == 2 && angles[0] - angles[1] > M_PI)) {
			type = 5;
			angles[1] += M_PI * 2;
			angles[2] += M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] >= 0 && angles[0] < angles[2]) {
			type = 6;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] >= 0 && angles[0] >= angles[2]) {
			type = 7;
			angles[1] += M_PI * 2;
			angles[2] += M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] >= 0 && angles[2] < 0 && angles[0] < angles[1]) {
			type = 8;
			angles[2] += M_PI * 2;
		}

		if (angles[2] < angles[0]) {
			kinematics.invertSpeed();
		}

		// initialize the visited flag
		std::vector<bool> visited(angles.size(), false);
		visited[0] = true;
		int unvisited = 2;

		// run forward until collision is deteted or all the poses are reached
		while (true) {
			try {
				kinematics.stepForward(2, false);
			}
			catch (char* ex) {
				// if only some of the poses are reached before collision, the collision is detected.
				kinematics.clear();
				return true;
			}

			// calculate the angle of the driving crank
			double angle = atan2(kinematics.diagram.joints[2]->pos.y - points[0].y, kinematics.diagram.joints[2]->pos.x - points[0].x);

			// convert the sign of the angle
			if (type == 1 && angle > 0) {
				angle -= M_PI * 2;
			}
			else if (type == 2 && angle > angles[0]) {
				angle -= M_PI * 2;
			}
			else if (type == 3 && angle < angles[0]) {
				angle += M_PI * 2;
			}
			else if (type == 4 && angle > 0) {
				angle -= M_PI * 2;
			}
			else if (type == 5 && angle < 0) {
				angle += M_PI * 2;
			}
			else if (type == 6 && angle > angles[0]) {
				angle -= M_PI * 2;
			}
			else if (type == 7 && angle < angles[0]) {
				angle += M_PI * 2;
			}
			else if (type == 8 && angle < 0) {
				angle += M_PI * 2;
			}

			// check if the poses are reached
			for (int i = 0; i < angles.size(); i++) {
				if (visited[i]) continue;

				if (angles[2] >= angles[0]) {
					if (angle >= angles[i]) {
						visited[i] = true;
						unvisited--;
					}
				}
				else {
					if (angle <= angles[i]) {
						visited[i] = true;
						unvisited--;
					}
				}
			}

			// if all the poses are reached without collision, no collision is detected.
			if (unvisited == 0) {
				kinematics.clear();
				return false;
			}
		}

		kinematics.clear();
		return false;
	}

	Kinematics LinkageSynthesisWattI::recordCollisionForConnectors(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points, const std::vector<Object25D> fixed_bodies, const std::vector<Object25D>& moving_bodies) {
		std::vector<glm::dvec2> connector_pts;
		Kinematics kinematics = constructKinematics(poses, points, {}, moving_bodies, true, fixed_bodies, connector_pts);
		kinematics.diagram.initialize();

		// calculate the rotational angle of the driving crank for 1st, 2nd, and last poses
		// i.e., angles[0] = first pose, angles[1] = second pose, angles[2] = last pose
		std::vector<double> angles(3);
		glm::dvec2 w(glm::inverse(poses[0][0]) * glm::dvec3(points[2], 1));
		for (int i = 0; i < 2; i++) {
			glm::dvec2 W = glm::dvec2(poses[0][i] * glm::dvec3(w, 1));
			angles[i] = atan2(W.y - points[0].y, W.x - points[0].x);
		}
		{
			glm::dvec2 W = glm::dvec2(poses[0].back() * glm::dvec3(w, 1));
			angles[2] = atan2(W.y - points[0].y, W.x - points[0].x);
		}

		// order the angles based on their signs
		int type = 0;
		if (angles[0] < 0 && angles[1] < 0 && angles[2] >= 0 && angles[0] >= angles[1]) {
			type = 1;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] < 0 && angles[0] < angles[2]) {
			type = 2;
			angles[1] -= M_PI * 2;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] < 0 && angles[0] >= angles[2]) {
			type = 3;
			angles[2] += M_PI * 2;
		}
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] >= 0 && (poses.size() >= 3 && angles[1] >= angles[2] || poses.size() == 2 && angles[1] - angles[0] > M_PI)) {
			type = 4;
			angles[1] -= M_PI * 2;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] < 0 && (poses.size() >= 3 && angles[1] < angles[2] || poses.size() == 2 && angles[0] - angles[1] > M_PI)) {
			type = 5;
			angles[1] += M_PI * 2;
			angles[2] += M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] >= 0 && angles[0] < angles[2]) {
			type = 6;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] >= 0 && angles[0] >= angles[2]) {
			type = 7;
			angles[1] += M_PI * 2;
			angles[2] += M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] >= 0 && angles[2] < 0 && angles[0] < angles[1]) {
			type = 8;
			angles[2] += M_PI * 2;
		}

		if (angles[2] < angles[0]) {
			kinematics.invertSpeed();
		}

		// initialize the visited flag
		std::vector<bool> visited(angles.size(), false);
		visited[0] = true;
		int unvisited = 2;

		// check the collision at the initial pose
		kinematics.diagram.recordCollisionForConnectors();

		// run forward until collision is deteted or all the poses are reached
		while (true) {
			try {
				kinematics.stepForward(3, false);
			}
			catch (char* ex) {
				// if only some of the poses are reached before collision, the collision is detected.
				return kinematics;
			}

			// calculate the angle of the driving crank
			double angle = atan2(kinematics.diagram.joints[2]->pos.y - points[0].y, kinematics.diagram.joints[2]->pos.x - points[0].x);

			// convert the sign of the angle
			if (type == 1 && angle > 0) {
				angle -= M_PI * 2;
			}
			else if (type == 2 && angle > angles[0]) {
				angle -= M_PI * 2;
			}
			else if (type == 3 && angle < angles[0]) {
				angle += M_PI * 2;
			}
			else if (type == 4 && angle > 0) {
				angle -= M_PI * 2;
			}
			else if (type == 5 && angle < 0) {
				angle += M_PI * 2;
			}
			else if (type == 6 && angle > angles[0]) {
				angle -= M_PI * 2;
			}
			else if (type == 7 && angle < angles[0]) {
				angle += M_PI * 2;
			}
			else if (type == 8 && angle < 0) {
				angle += M_PI * 2;
			}

			// check if the poses are reached
			for (int i = 0; i < angles.size(); i++) {
				if (visited[i]) continue;

				if (angles[2] >= angles[0]) {
					if (angle >= angles[i]) {
						visited[i] = true;
						unvisited--;
					}
				}
				else {
					if (angle <= angles[i]) {
						visited[i] = true;
						unvisited--;
					}
				}
			}

			// if all the poses are reached without collision, no collision is detected.
			if (unvisited == 0) {
				return kinematics;
			}
		}

		return kinematics;
	}

	/**
	 * Calculate the deviation of the link lengths across the poses.
	 *
	 * @param poses		poses
	 * @param points	the coordinates of the joints at the first pose
	 * @return			square root of the sum of the devation of the link lengths
	 */
	double LinkageSynthesisWattI::check(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points) {
		glm::dvec2 P0 = points[0];
		glm::dvec2 P1 = points[1];
		glm::dvec2 P2 = points[2];
		glm::dvec2 P3 = points[3];
		glm::dvec2 P4 = points[4];
		glm::dvec2 P5 = points[5];
		glm::dvec2 P6 = points[6];

		glm::dvec2 p2(glm::inverse(poses[0][0]) * glm::dvec3(P2, 1));
		glm::dvec2 p3(glm::inverse(poses[0][0]) * glm::dvec3(P3, 1));
		glm::dvec2 p4(glm::inverse(poses[1][0]) * glm::dvec3(P4, 1));
		glm::dvec2 p5(glm::inverse(poses[0][0]) * glm::dvec3(P5, 1));
		glm::dvec2 p6(glm::inverse(poses[1][0]) * glm::dvec3(P6, 1));
		
		std::vector<double> lengths;
		lengths.push_back(glm::length(P2 - P0));
		lengths.push_back(glm::length(P3 - P1));
		lengths.push_back(glm::length(P3 - P2));
		lengths.push_back(glm::length(P5 - P2));
		lengths.push_back(glm::length(P5 - P3));
		lengths.push_back(glm::length(P4 - P1));
		lengths.push_back(glm::length(P4 - P3));
		lengths.push_back(glm::length(P4 - P3));
		lengths.push_back(glm::length(P6 - P4));
		lengths.push_back(glm::length(P6 - P5));

		double ans = 0.0;
		for (int i = 1; i < poses.size(); i++) {
			glm::dvec2 P2b(poses[0][i] * glm::dvec3(p2, 1));
			glm::dvec2 P3b(poses[0][i] * glm::dvec3(p3, 1));
			glm::dvec2 P4b(poses[1][i] * glm::dvec3(p4, 1));
			glm::dvec2 P5b(poses[0][i] * glm::dvec3(p5, 1));
			glm::dvec2 P6b(poses[1][i] * glm::dvec3(p6, 1));

			std::vector<double> lengths2;
			lengths2.push_back(glm::length(P2b - P0));
			lengths2.push_back(glm::length(P3b - P1));
			lengths2.push_back(glm::length(P3b - P2b));
			lengths2.push_back(glm::length(P5b - P2b));
			lengths2.push_back(glm::length(P5b - P3b));
			lengths2.push_back(glm::length(P4b - P1));
			lengths2.push_back(glm::length(P4b - P3b));
			lengths2.push_back(glm::length(P4b - P3b));
			lengths2.push_back(glm::length(P6b - P4b));
			lengths2.push_back(glm::length(P6b - P5b));

			for (int i = 0; i < lengths.size(); i++) {
				ans += (lengths[i] - lengths2[i]) * (lengths[i] - lengths2[i]);
			}
		}

		return std::sqrt(ans);
	}

	void LinkageSynthesisWattI::generate3DGeometry(const Kinematics& kinematics, std::vector<Vertex>& vertices) {
		// generate geometry of rigid bodies
		for (int j = 0; j < kinematics.diagram.bodies.size(); j++) {
			for (int k = 0; k < kinematics.diagram.bodies[j]->size(); k++) {
				std::vector<glm::dvec2> points = kinematics.diagram.bodies[j]->getActualPoints(k);
				std::vector<glm::dvec2> points2 = kinematics.diagram.bodies[j]->getActualPoints2(k);
				float z = kinematics.diagram.bodies[j]->polygons[k].depth1;
				float depth = kinematics.diagram.bodies[j]->polygons[k].depth2 - kinematics.diagram.bodies[j]->polygons[k].depth1;
				glutils::drawPrism(points, points2, depth, glm::vec4(0.7, 1, 0.7, 1), glm::translate(glm::mat4(), glm::vec3(0, 0, z)), vertices);
			}
		}

		// generate geometry of links
		for (int j = 0; j < kinematics.diagram.links.size(); j++) {
			// For the coupler, we can use the moving body itself as a coupler, 
			// so we do not need to create a coupler link.
			if (!kinematics.diagram.links[j]->actual_link) continue;

			if (kinematics.diagram.links[j]->joints.size() == 2) {
				glm::dvec2& p1 = kinematics.diagram.links[j]->joints[0]->pos;
				glm::dvec2& p2 = kinematics.diagram.links[j]->joints[1]->pos;
				std::vector<glm::dvec2> pts = generateRoundedBarPolygon(p1, p2, options->link_width / 2);
				float z = kinematics.diagram.links[j]->z * (options->link_depth + options->gap * 2 + options->joint_cap_depth) - options->link_depth;
				glutils::drawPrism(pts, options->link_depth, glm::vec4(0.7, 0.7, 0.7, 1), glm::translate(glm::mat4(), glm::vec3(0, 0, z)), vertices);
				glutils::drawPrism(pts, options->link_depth, glm::vec4(0.7, 0.7, 0.7, 1), glm::translate(glm::mat4(), glm::vec3(0, 0, -options->body_depth - z - options->link_depth)), vertices);
			}
			else if (kinematics.diagram.links[j]->joints.size() == 3) {
				float z = kinematics.diagram.links[j]->z * (options->link_depth + options->gap * 2 + options->joint_cap_depth) - options->link_depth;
				std::vector<glm::dvec2> polygon = generateRoundedTrianglePolygon({ kinematics.diagram.links[j]->joints[0]->pos, kinematics.diagram.links[j]->joints[1]->pos, kinematics.diagram.links[j]->joints[2]->pos }, options->link_width / 2);
				glutils::drawPrism(polygon, options->link_depth, glm::vec4(0.7, 0.7, 0.7, 1), glm::translate(glm::mat4(), glm::vec3(0, 0, z)), vertices);
				glutils::drawPrism(polygon, options->link_depth, glm::vec4(0.7, 0.7, 0.7, 1), glm::translate(glm::mat4(), glm::vec3(0, 0, -options->body_depth - z - options->link_depth)), vertices);
			}
		}

		// generate geometry of joint [3], which is between two ternary links
		generateJointGeometry(kinematics.diagram.joints[3]->pos, std::min(kinematics.diagram.links[1]->z, kinematics.diagram.links[2]->z), std::max(kinematics.diagram.links[1]->z, kinematics.diagram.links[2]->z), glm::vec4(0.7, 1, 0.7, 1), vertices);

		// generate geometry of joints [2] and [5]
		// Temporarly implementation
		// We should use the z data from diagmra.connectors
		std::vector<int> zs2 = { kinematics.diagram.connectors[2].z, kinematics.diagram.links[0]->z, kinematics.diagram.links[2]->z };
		std::sort(zs2.begin(), zs2.end());
		generateJointGeometry(kinematics.diagram.joints[2]->pos, zs2[1], zs2[2], glm::vec4(0.7, 1, 0.7, 1), vertices);
		generateJointGeometry(kinematics.diagram.joints[2]->pos, zs2[1], zs2[0], glm::vec4(0.7, 1, 0.7, 1), vertices);

		std::vector<int> zs5 = { kinematics.diagram.connectors[3].z, kinematics.diagram.links[2]->z, kinematics.diagram.links[4]->z };
		std::sort(zs5.begin(), zs5.end());
		generateJointGeometry(kinematics.diagram.joints[5]->pos, zs5[1], zs5[2], glm::vec4(0.7, 1, 0.7, 1), vertices);
		generateJointGeometry(kinematics.diagram.joints[5]->pos, zs5[1], zs5[0], glm::vec4(0.7, 1, 0.7, 1), vertices);
	}

	void LinkageSynthesisWattI::generateJointGeometry(const glm::dvec2& pos, int bottom_z, int top_z, const glm::vec4& color, std::vector<Vertex>& vertices) {
		if (top_z >= bottom_z) {
			double z = bottom_z * (options->link_depth + options->gap * 2 + options->joint_cap_depth);
			double height = (top_z - bottom_z) * (options->link_depth + options->gap * 2 + options->joint_cap_depth) - options->link_depth - options->gap;
			glutils::drawCylinderZ(options->link_width / 2, options->link_width / 2, options->link_width / 2, options->link_width / 2, height, color, glm::translate(glm::mat4(), glm::vec3(pos, z)), vertices);
			glutils::drawCylinderZ(options->link_width / 2, options->link_width / 2, options->link_width / 2, options->link_width / 2, height, color, glm::translate(glm::mat4(), glm::vec3(pos, -options->body_depth - z - height)), vertices);

			z += height;
			height = options->link_depth + options->gap * 2;
			glutils::drawCylinderZ(options->joint_radius, options->joint_radius, options->joint_radius, options->joint_radius, height, color, glm::translate(glm::mat4(), glm::vec3(pos, z)), vertices);
			glutils::drawCylinderZ(options->joint_radius, options->joint_radius, options->joint_radius, options->joint_radius, height, color, glm::translate(glm::mat4(), glm::vec3(pos, -options->body_depth - z - height)), vertices);

			z += height;
			height = options->joint_cap_depth;
			glutils::drawCylinderZ(options->joint_cap_radius2, options->joint_cap_radius2, options->joint_cap_radius1, options->joint_cap_radius1, height, color, glm::translate(glm::mat4(), glm::vec3(pos, z)), vertices);
			glutils::drawCylinderZ(options->joint_cap_radius1, options->joint_cap_radius1, options->joint_cap_radius2, options->joint_cap_radius2, height, color, glm::translate(glm::mat4(), glm::vec3(pos, -options->body_depth - z - height)), vertices);
		}
		else {
			double z = top_z * (options->link_depth + options->gap * 2 + options->joint_cap_depth) + options->gap;
			double height = (bottom_z - top_z) * (options->link_depth + options->gap * 2 + options->joint_cap_depth) - options->link_depth - options->gap;
			glutils::drawCylinderZ(options->link_width / 2, options->link_width / 2, options->link_width / 2, options->link_width / 2, height, color, glm::translate(glm::mat4(), glm::vec3(pos, z)), vertices);
			glutils::drawCylinderZ(options->link_width / 2, options->link_width / 2, options->link_width / 2, options->link_width / 2, height, color, glm::translate(glm::mat4(), glm::vec3(pos, -options->body_depth - z - height)), vertices);

			height = options->link_depth + options->gap * 2;
			z -= height;
			glutils::drawCylinderZ(options->joint_radius, options->joint_radius, options->joint_radius, options->joint_radius, height, color, glm::translate(glm::mat4(), glm::vec3(pos, z)), vertices);
			glutils::drawCylinderZ(options->joint_radius, options->joint_radius, options->joint_radius, options->joint_radius, height, color, glm::translate(glm::mat4(), glm::vec3(pos, -options->body_depth - z - height)), vertices);

			height = options->joint_cap_depth;
			z -= height;
			glutils::drawCylinderZ(options->joint_cap_radius1, options->joint_cap_radius1, options->joint_cap_radius2, options->joint_cap_radius2, height, color, glm::translate(glm::mat4(), glm::vec3(pos, z)), vertices);
			glutils::drawCylinderZ(options->joint_cap_radius2, options->joint_cap_radius2, options->joint_cap_radius1, options->joint_cap_radius1, height, color, glm::translate(glm::mat4(), glm::vec3(pos, -options->body_depth - z - height)), vertices);
		}
	}

	void LinkageSynthesisWattI::saveSTL(const QString& dirname, const std::vector<Kinematics>& kinematics) {
	}

	void LinkageSynthesisWattI::saveSCAD(const QString& dirname, int index, const Kinematics& kinematics) {
		// generate geometry of rigid bodies
		for (int j = 0; j < kinematics.diagram.bodies.size(); j++) {
			if (j == 0) {
				// HACK: for the first moving body, we need to add a hole for the joint connectors
				std::vector<std::vector<glm::dvec3>> holes(4);
				for (int hi = 0; hi < 4; hi++) {
					glm::dvec2 center = (hi < 2) ? kinematics.diagram.joints[2]->pos : kinematics.diagram.joints[5]->pos;
					double z;
					if (hi == 0) z = kinematics.diagram.connectors[2].z * (options->link_depth + options->gap * 2 + options->joint_cap_depth) - options->link_depth - 0.1;
					else if (hi == 1) z = -options->body_depth - kinematics.diagram.connectors[2].z * (options->link_depth + options->gap * 2 + options->joint_cap_depth) - 0.1;
					else if (hi == 2) z = kinematics.diagram.connectors[3].z * (options->link_depth + options->gap * 2 + options->joint_cap_depth) - options->link_depth - 0.1;
					else z = -options->body_depth - kinematics.diagram.connectors[3].z * (options->link_depth + options->gap * 2 + options->joint_cap_depth) - 0.1;

					holes[hi].resize(32);
					for (int k = 0; k < 32; k++) {
						double theta = (double)k / 32 * M_PI * 2;
						holes[hi][k] = glm::dvec3(center.x + cos(theta) * options->hole_radius, center.y + sin(theta) * options->hole_radius, z);
					}
				}
				QString name = QString("body_%1_%2").arg(index).arg(j);
				QString filename = dirname + "/" + name + ".scad";
				SCADExporter::save(filename, name, kinematics.diagram.bodies[j], holes, options->link_depth + 0.2);
			}
			else {
				QString name = QString("body_%1_%2").arg(index).arg(j);
				QString filename = dirname + "/" + name + ".scad";
				SCADExporter::save(filename, name, kinematics.diagram.bodies[j]);
			}
		}

		// generate geometry of links
		for (int j = 0; j < kinematics.diagram.links.size(); j++) {
			// For the coupler, we can use the moving body itself as a coupler, 
			// so we do not need to create a coupler link.
			if (!kinematics.diagram.links[j]->actual_link) continue;

			if (kinematics.diagram.links[j]->joints.size() == 2) {
				// this is for link 0 and 3
				glm::dvec2& p1 = kinematics.diagram.links[j]->joints[0]->pos;
				glm::dvec2& p2 = kinematics.diagram.links[j]->joints[1]->pos;
				std::vector<glm::dvec2> pts = generateRoundedBarPolygon(glm::vec2(), p2 - p1, options->link_width / 2);
				std::vector<std::vector<glm::dvec2>> holes(2);
				holes[0] = generateCirclePolygon(glm::vec2(), options->hole_radius);
				holes[1] = generateCirclePolygon(p2 - p1, options->hole_radius);

				QString name = QString("link_%1_%2").arg(index).arg(j);
				QString filename = dirname + "/" + name + ".scad";
				SCADExporter::save(filename, name, pts, holes, options->link_depth);
			}
			else if (kinematics.diagram.links[j]->joints.size() == 3 && j == 1) {
				// this is for link 1
				std::vector<glm::dvec2> pts = generateRoundedTrianglePolygon({ kinematics.diagram.links[j]->joints[0]->pos, kinematics.diagram.links[j]->joints[1]->pos, kinematics.diagram.links[j]->joints[2]->pos }, options->link_width / 2);

				std::vector<std::vector<glm::dvec2>> holes(3);
				holes[0] = generateCirclePolygon(kinematics.diagram.links[j]->joints[0]->pos, options->hole_radius);
				holes[1] = generateCirclePolygon(kinematics.diagram.links[j]->joints[1]->pos, options->hole_radius);
				holes[2] = generateCirclePolygon(kinematics.diagram.links[j]->joints[2]->pos, options->hole_radius);

				QString name = QString("link_%1_%2").arg(index).arg(j);
				QString filename = dirname + "/" + name + ".scad";
				SCADExporter::save(filename, name, pts, holes, options->link_depth);
			}
			else if (kinematics.diagram.links[j]->joints.size() == 3) {
				// this is for link 2
				std::vector<glm::dvec2> pts = generateRoundedTrianglePolygon({ kinematics.diagram.links[j]->joints[0]->pos, kinematics.diagram.links[j]->joints[1]->pos, kinematics.diagram.links[j]->joints[2]->pos }, options->link_width / 2);
								
				std::vector<int> zs2 = { kinematics.diagram.connectors[2].z, kinematics.diagram.links[0]->z, kinematics.diagram.links[2]->z };
				std::sort(zs2.begin(), zs2.end());
				std::vector<int> zs5 = { kinematics.diagram.connectors[3].z, kinematics.diagram.links[2]->z, kinematics.diagram.links[4]->z };
				std::sort(zs5.begin(), zs5.end());

				std::vector<Polygon25D> polygons;
				// joint [2]
				generateJointGeometry(kinematics.diagram.links[j]->joints[0]->pos, zs2[1], zs2[2], polygons);
				generateJointGeometry(kinematics.diagram.links[j]->joints[0]->pos, zs2[1], zs2[0], polygons);

				// joint [3]
				generateJointGeometry(kinematics.diagram.links[j]->joints[1]->pos, kinematics.diagram.links[2]->z, kinematics.diagram.links[1]->z, polygons);

				// joint [5]
				generateJointGeometry(kinematics.diagram.links[j]->joints[2]->pos, zs5[1], zs5[2], polygons);
				generateJointGeometry(kinematics.diagram.links[j]->joints[2]->pos, zs5[1], zs5[0], polygons);

				QString name = QString("link_%1_%2").arg(index).arg(j);
				QString filename = dirname + "/" + name + ".scad";
				SCADExporter::save(filename, name, pts, options->link_depth, polygons);

				/////////////////////////////////////////////////////////
				// vertical flip for the other side

				{
					// this is for link 2
					std::vector<glm::dvec2> pts = generateRoundedTrianglePolygon({ kinematics.diagram.links[j]->joints[0]->pos, kinematics.diagram.links[j]->joints[1]->pos, kinematics.diagram.links[j]->joints[2]->pos }, options->link_width / 2);

					std::vector<int> zs2 = { kinematics.diagram.connectors[2].z, kinematics.diagram.links[0]->z, kinematics.diagram.links[2]->z };
					std::sort(zs2.begin(), zs2.end());
					std::vector<int> zs5 = { kinematics.diagram.connectors[3].z, kinematics.diagram.links[2]->z, kinematics.diagram.links[4]->z };
					std::sort(zs5.begin(), zs5.end());

					std::vector<Polygon25D> polygons;
					// joint [2]
					generateJointGeometry(kinematics.diagram.links[j]->joints[0]->pos, zs2[1], zs2[2], polygons);
					generateJointGeometry(kinematics.diagram.links[j]->joints[0]->pos, zs2[1], zs2[0], polygons);

					// joint [3]
					generateJointGeometry(kinematics.diagram.links[j]->joints[1]->pos, kinematics.diagram.links[2]->z, kinematics.diagram.links[1]->z, polygons);

					// joint [5]
					generateJointGeometry(kinematics.diagram.links[j]->joints[2]->pos, zs5[1], zs5[2], polygons);
					generateJointGeometry(kinematics.diagram.links[j]->joints[2]->pos, zs5[1], zs5[0], polygons);

					QString name = QString("link_%1_%2_flipped").arg(index).arg(j);
					QString filename = dirname + "/" + name + ".scad";
					SCADExporter::save(filename, name, pts, options->link_depth, polygons);
				}
			}
		}
	}

	void LinkageSynthesisWattI::generateJointGeometry(const glm::dvec2& pos, int bottom_z, int top_z, std::vector<Polygon25D>& polygons) {
		if (bottom_z <= top_z) {
			double z = options->link_depth;
			double height = (top_z - bottom_z) * (options->link_depth + options->gap * 2 + options->joint_cap_depth) - options->link_depth - options->gap;
			std::vector<glm::dvec2> pts = generateCirclePolygon(pos, options->link_width / 2);
			polygons.push_back(Polygon25D(pts, z, z + height, false));

			z += height;
			height = options->link_depth + options->gap * 2;
			pts = generateCirclePolygon(pos, options->hole_radius);
			polygons.push_back(Polygon25D(pts, z, z + height, false));

			z += height;
			height = options->joint_cap_depth;
			pts = generateCirclePolygon(pos, options->joint_cap_radius1);
			std::vector<glm::dvec2> pts2 = generateCirclePolygon(pos, options->joint_cap_radius2);
			polygons.push_back(Polygon25D(pts2, pts, z, z + height, false));
		}
		else {
			double z = 0;
			double height = (bottom_z - top_z) * (options->link_depth + options->gap * 2 + options->joint_cap_depth) - options->link_depth - options->gap;
			std::vector<glm::dvec2> pts = generateCirclePolygon(pos, options->link_width / 2);
			polygons.push_back(Polygon25D(pts, z - height, z, false));

			z -= height;
			height = options->link_depth + options->gap * 2;
			pts = generateCirclePolygon(pos, options->hole_radius);
			polygons.push_back(Polygon25D(pts, z - height, z, false));

			z -= height;
			height = options->joint_cap_depth;
			pts = generateCirclePolygon(pos, options->joint_cap_radius1);
			std::vector<glm::dvec2> pts2 = generateCirclePolygon(pos, options->joint_cap_radius2);
			polygons.push_back(Polygon25D(pts, pts2, z - height, z, false));
		}
	}

}
