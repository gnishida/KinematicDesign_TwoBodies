#include "LeastSquareSolver.h"
#include "KinematicUtils.h"

namespace kinematics {

	SolverForLink::SolverForLink(const std::vector<glm::dmat3x3>& poses) {
		this->poses = poses;
	}

	double SolverForLink::operator() (const column_vector& arg) const {
		glm::dvec2 A0(arg(0, 0), arg(1, 0));
		glm::dvec2 a(arg(2, 0), arg(3, 0));

		glm::dvec2 A1(poses[0] * glm::dvec3(a, 1));
		double l1_squared = glm::length(A1 - A0);
		l1_squared = l1_squared * l1_squared;

		double ans = 0.0;
		for (int i = 1; i < poses.size(); i++) {
			glm::dvec2 A(poses[i] * glm::dvec3(a, 1));
			double l_squared = glm::length(A - A0);
			l_squared = l_squared * l_squared;
			ans += (l_squared - l1_squared) * (l_squared - l1_squared);
		}

		return ans;
	}

	SolverForSlider::SolverForSlider(const std::vector<glm::dmat3x3>& poses) {
		this->poses = poses;
	}

	double SolverForSlider::operator() (const column_vector& arg) const {
		glm::dvec2 a(arg(0, 0), arg(1, 0));

		glm::dvec2 A1(poses[0] * glm::dvec3(a, 1));
		glm::dvec2 A2(poses[1] * glm::dvec3(a, 1));

		glm::dvec2 v1 = A2 - A1;
		v1 /= glm::length(v1);

		double ans = 0.0;
		for (int i = 2; i < poses.size(); i++) {
			glm::dvec2 A(poses[i] * glm::dvec3(a, 1));
			glm::dvec2 v = A - A1;
			v /= glm::length(v);

			ans += abs(crossProduct(v1, v));
		}

		return ans;
	}

	SolverForWattI::SolverForWattI(const std::vector<std::vector<glm::dmat3x3>>& poses) {
		this->poses = poses;
	}

	double SolverForWattI::operator() (const column_vector& arg) const {
		glm::dvec2 P0(arg(0, 0), arg(1, 0));
		glm::dvec2 P1(arg(2, 0), arg(3, 0));
		glm::dvec2 P2(arg(4, 0), arg(5, 0));
		glm::dvec2 P3(arg(6, 0), arg(7, 0));
		glm::dvec2 P4(arg(8, 0), arg(9, 0));
		glm::dvec2 P5(arg(10, 0), arg(11, 0));
		glm::dvec2 P6(arg(12, 0), arg(13, 0));
		
		std::vector<double> lengths;
		lengths.push_back(glm::length2(P2 - P0));
		lengths.push_back(glm::length2(P3 - P1));
		lengths.push_back(glm::length2(P6 - P5));
		lengths.push_back(glm::length2(P4 - P3));
		lengths.push_back(glm::length2(P4 - P1));

		glm::dvec2 p2(glm::inverse(poses[0][0]) * glm::dvec3(P2, 1));
		glm::dvec2 p3(glm::inverse(poses[0][0]) * glm::dvec3(P3, 1));
		glm::dvec2 p4(glm::inverse(poses[1][0]) * glm::dvec3(P4, 1));
		glm::dvec2 p5(glm::inverse(poses[0][0]) * glm::dvec3(P5, 1));
		glm::dvec2 p6(glm::inverse(poses[1][0]) * glm::dvec3(P6, 1));

		double ans = 0.0;
		for (int i = 1; i < poses.size(); i++) {
			glm::dvec2 P2b(poses[0][i] * glm::dvec3(p2, 1));
			glm::dvec2 P3b(poses[0][i] * glm::dvec3(p3, 1));
			glm::dvec2 P4b(poses[1][i] * glm::dvec3(p4, 1));
			glm::dvec2 P5b(poses[0][i] * glm::dvec3(p5, 1));
			glm::dvec2 P6b(poses[1][i] * glm::dvec3(p6, 1));

			std::vector<double> lengths2;
			lengths2.push_back(glm::length2(P2b - P0));
			lengths2.push_back(glm::length2(P3b - P1));
			lengths2.push_back(glm::length2(P6b - P5b));
			lengths2.push_back(glm::length2(P4b - P3b));
			lengths2.push_back(glm::length2(P4b - P1));

			for (int i = 0; i < lengths.size(); i++) {
				ans += (lengths[i] - lengths2[i]) * (lengths[i] - lengths2[i]);
			}
		}

		return ans;
	}

}