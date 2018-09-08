#pragma once

#include <vector>
#include <glm/glm.hpp>
#include "LinkageSynthesis.h"
#include "Solution.h"
#include "BBox.h"
#include "LeastSquareSolver.h"

namespace kinematics {

	class LinkageSynthesisWattI : public LinkageSynthesis {
	public:
		LinkageSynthesisWattI(const std::vector<Object25D>& fixed_bodies, const std::pair<double, double>& sigmas, bool avoid_branch_defect, double min_transmission_angle, double min_link_length, const std::vector<double>& weights);

	public:
		void calculateSolution(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const std::vector<glm::dvec2>& linkage_avoidance_pts, int num_samples, const std::vector<Object25D>& moving_bodies, std::vector<Solution>& solutions);
		bool optimizeCandidate(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const BBox& bbox, std::vector<glm::dvec2>& points);
		bool optimizeLink(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const BBox& bbox, std::vector<glm::dvec2>& points);
		Solution findBestSolution(const std::vector<std::vector<glm::dmat3x3>>& poses, std::vector<Solution>& solutions, const std::vector<glm::dvec2>& linkage_region_pts, const cv::Mat& dist_map, const BBox& dist_map_bbox, const std::vector<glm::dvec2>& linkage_avoidance_pts, const std::vector<Object25D>& moving_bodies, int num_particles, int num_iterations, bool record_file);
		double calculateCost(Solution& solution, const std::vector<Object25D>& moving_bodies, const cv::Mat& dist_map, const BBox& dist_map_bbox);
		int getType(const std::vector<glm::dvec2>& points);
		bool checkRotatableCrankDefect(const std::vector<glm::dvec2>& points);
		std::pair<double, double> checkRange(const std::vector<glm::dvec2>& points);
		bool checkOrderDefect(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points);
		bool checkBranchDefect(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points);
		bool checkCircuitDefect(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points);
		Kinematics constructKinematics(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points, const std::vector<std::vector<int>>& zorder, const std::vector<Object25D>& moving_bodies, bool connect_joints, const std::vector<Object25D>& fixed_bodies, std::vector<glm::dvec2>& connected_pts);
		void updateMovingBodies(Kinematics& kin, const std::vector<Object25D>& moving_bodies);

		bool checkHardConstraints(std::vector<glm::dvec2>& points, const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const std::vector<glm::dvec2>& linkage_avoidance_pts, const std::vector<Object25D>& moving_bodies, std::vector<std::vector<int>>& zorder, double simulation_speed);
		bool checkCollision(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points, const std::vector<Object25D>& fixed_bodies, const std::vector<Object25D>& moving_bodies, double simulation_speed);
		Kinematics recordCollisionForConnectors(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points, const std::vector<Object25D> fixed_bodies, const std::vector<Object25D>& moving_bodies, double simulation_speed);
		double tortuosityOfTrajectory(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points, const std::vector<Object25D>& moving_bodies, double simulation_speed);
		double check(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points);

		void generate3DGeometry(const Kinematics& kinematics, std::vector<Vertex>& vertices);
		void generateJointGeometry(const glm::dvec2& pos, int bottom_z, int top_z, const glm::vec4& color, std::vector<Vertex>& vertices);
		void saveSCAD(const QString& dirname, int index, const Kinematics& kinematics);
		void saveSTL(const QString& dirname, const std::vector<Kinematics>& kinematics);
		void generateJointGeometry(const glm::dvec2& pos, int bottom_z, int top_z, std::vector<Polygon25D>& polygons);
	};

}
