#ifndef LSC_PLANNER_BERNSTEIN_TRAJECTORY_HPP
#define LSC_PLANNER_BERNSTEIN_TRAJECTORY_HPP

#include <queue>
#include <sp_const.hpp>
#include <Eigen/Dense>

namespace DynamicPlanning{
    static int nChoosek(int n, int k){
        if(k > n) return 0;
        if(k * 2 > n) k = n-k;
        if(k == 0) return 1;

        int result = n;
        for(int i = 2; i <= k; i++){
            result *= (n-i+1);
            result /= i;
        }
        return result;
    }

    static double getBernsteinBasis(int n, int i, double t_normalized){
        return nChoosek(n, i) * pow(t_normalized, i) * pow(1-t_normalized, n - i);
    }

    //TODO: MINVO basis

    static point3d getPointFromControlPoints(points_t control_points,
                                             double t_normalized){
        point3d point;
        double t = t_normalized;
        if(t < 0 - SP_EPSILON || t > 1 + SP_EPSILON){
            throw std::invalid_argument("[Polynomial] Input of getPointFromControlPoints is out of bound");
        }

        int n_ctrl = (int)control_points.size() - 1;
        double x = 0, y = 0, z = 0;
        for(int i = 0; i < n_ctrl + 1; i++){
            double b_i_n = getBernsteinBasis(n_ctrl, i, t_normalized);
            x += control_points[i].x() * b_i_n;
            y += control_points[i].y() * b_i_n;
            z += control_points[i].z() * b_i_n;
        }

        point = point3d((float)x, (float)y, (float)z);
        return point;
    }

    static double getPointFromControlPoints(std::vector<double> control_points, double t_normalized){
        double t = t_normalized;
        if(t < 0 - SP_EPSILON || t > 1 + SP_EPSILON){
            throw std::invalid_argument("[Polynomial] Input of getPointFromControlPoints is out of bound");
        }

        int n_ctrl = (int)control_points.size() - 1;
        double x = 0;
        for(int i = 0; i < n_ctrl + 1; i++){
            double b_i_n = getBernsteinBasis(n_ctrl, i, t_normalized);
            x += control_points[i] * b_i_n;
        }

        return x;
    }

    static points_t bernsteinFitting(points_t target_points, std::vector<double> ts_normalized){
        int n = (int)target_points.size() - 1;
        Eigen::MatrixXd B = Eigen::MatrixXd(n+1, n+1);
        for(int i = 0; i < n + 1; i++){
            for(int j = 0; j < n + 1; j++){
                B(i, j) = getBernsteinBasis(n, j, ts_normalized[i]);
            }
        }

        Eigen::MatrixXd X = Eigen::MatrixXd(n+1, 3);
        for(int i = 0; i < n + 1; i++){
            X(i, 0) = target_points[i].x();
            X(i, 1) = target_points[i].y();
            X(i, 2) = target_points[i].z();
        }

        Eigen::MatrixXd C = B.inverse() * X;
        points_t control_points;
        control_points.resize(n+1);
        for(int i = 0; i < n + 1; i++){
            control_points[i] = point3d(C(i, 0), C(i, 1), C(i, 2));
        }
        return control_points;
    }

    static int coef_derivative(int n, int phi){
        if(n < phi){
            return 0;
        }

        int coef = 1;
        for(int i = 0; i < phi; i++){
            coef *= n - i;
        }
        return coef;
    }

    struct BisectionElement{
        double c;
        int k;
        Eigen::VectorXd coef;
    };

    // return list of [a,b] includes one real root of polynomial which coefficient is coef
    static std::vector<std::pair<double, double>> realRootIsolation(const Eigen::VectorXd& coef, int n) {
        std::queue<BisectionElement> L;
        L.push(BisectionElement{0, 0, coef});
        std::vector<std::pair<double, double>> Isol;
        int n_poly = 2 * n - 1;

        while (not L.empty()) {
            BisectionElement current_element = L.front();
            L.pop();
//                std::cout << "current_coef: " << std::endl << current_element.coef << std::endl;
            if (current_element.coef(0) == 0) {
                for (int i = 0; i < n_poly; i++) {
                    current_element.coef(i) = current_element.coef(0, i + 1);
                }
                current_element.coef(n_poly) = 0;
                n_poly--;
                Isol.emplace_back(std::make_pair(current_element.c / pow(2, current_element.k),
                                                 current_element.c / pow(2, current_element.k)));
            }

            Eigen::VectorXd coef_test = Eigen::VectorXd::Zero(n_poly + 1);
            for (int i = 0; i < n_poly + 1; i++) {
                for (int j = 0; j < n_poly + 1 - i; j++) {
                    coef_test(j) += current_element.coef(i) * nChoosek(n_poly - i, j);
                }
            }
//                std::cout << "coef_test: " << std::endl << coef_test << std::endl;
            int var = 0;
            for (int i = 0; i < n_poly; i++) {
                if (coef_test(i) * coef_test(i + 1) < 0) {
                    var++;
                }
            }
            if (var == 1) {
                Isol.emplace_back(std::make_pair(current_element.c / pow(2, current_element.k),
                                                 (current_element.c + 1) / pow(2, current_element.k)));
            }
            if (var > 1) {
                Eigen::VectorXd coef1 = current_element.coef;
                for (int i = 0; i < n_poly + 1; i++) {
                    coef1(i) *= pow(2, n_poly - i);
                }
//                    std::cout << "coef1: " << std::endl << coef1 << std::endl;
                L.push(BisectionElement{2 * current_element.c, current_element.k + 1, coef1});

                Eigen::VectorXd coef2 = Eigen::VectorXd::Zero(n_poly + 1);
                for (int i = 0; i < n_poly + 1; i++) {
                    for (int j = 0; j < i + 1; j++) {
                        coef2(j) += current_element.coef(i) * pow(2, n_poly - i) * nChoosek(i, j);
                    }
                }
//                    std::cout << "coef2: " << std::endl << coef2 << std::endl;
                L.push(BisectionElement{2 * current_element.c + 1, current_element.k + 1, coef2});
            }
        }
        return Isol;
    }

    static double evalPoly(const Eigen::VectorXd& coef, double t){
        double ret = 0;
        for(int i = 0; i < coef.size(); i++){
            ret += coef(i) * pow(t, i);
        }
        return ret;
    }

    // pi_i = obstacle position, pi_j = agent_position
    static double distanceBetweenPolys(const points_t& control_points_agent,
                                       const points_t& control_points_obs,
                                       const Eigen::MatrixXd& B,
                                       double poly_root_tolerance,
                                       point3d& closest_point) {
        if(control_points_agent.size() != control_points_obs.size()){
            throw std::invalid_argument("[Polynomial] degree of two polynomials are not same.");
        }

        int n = (int)control_points_agent.size() - 1;
        int dim = 3;

        //get control points of relative trajectory
        points_t control_points_rel;
        control_points_rel.resize(n + 1);
        for(int i = 0; i < n + 1; i++){
            control_points_rel[i].x() = control_points_agent[i].x() - control_points_obs[i].x();
            control_points_rel[i].y() = control_points_agent[i].y() - control_points_obs[i].y();
            control_points_rel[i].z() = control_points_agent[i].z() - control_points_obs[i].z();
        }

        Eigen::MatrixXd control_points_rel_mtx = Eigen::MatrixXd::Zero(dim, n + 1);
        for(int j = 0; j < n + 1; j++){
            control_points_rel_mtx(0, j) = control_points_rel[j].x();
            control_points_rel_mtx(1, j) = control_points_rel[j].y();
            control_points_rel_mtx(2, j) = control_points_rel[j].z();
        }

        // convert to coef of standard basis
        Eigen::MatrixXd coef = control_points_rel_mtx * B;

        // get derivative of coef
        Eigen::MatrixXd coef_derivative = Eigen::MatrixXd::Zero(dim, n + 1);
        for(int j = 0; j < n; j++){
            coef_derivative(0, j) = (j + 1) * coef(0, j + 1);
            coef_derivative(1, j) = (j + 1) * coef(1, j + 1);
            coef_derivative(2, j) = (j + 1) * coef(2, j + 1);
        }

        // coef dot coef_derivative
        Eigen::VectorXd coef_g = Eigen::VectorXd::Zero(2 * n);
        for(int j0 = 0; j0 < n + 1; j0++){
            for(int j1 = 0; j1 < n; j1++){
                for(int k = 0; k < dim; k++) {
                    coef_g(j0 + j1) += coef(k, j0) * coef_derivative(k, j1);
                }
            }
        }

        // real root isolation
        std::vector<std::pair<double, double>> Isol = realRootIsolation(coef_g, n);

        // compute closest point by Newton method
        bool isClosestPointInSection = false;
        double a, b, m, g_m, t_cand, dist_cand;
        double dist_closest = SP_INFINITY;
        point3d p_cand;
        for(auto section : Isol){
            a = section.first;
            b = section.second;
            if(evalPoly(coef_g, a) < 0 and evalPoly(coef_g, b) > 0) {
                while (true) {
                    if (b - a < poly_root_tolerance) {
                        t_cand = (a + b) / 2;
                        break;
                    }
                    m = (a + b) / 2;
                    g_m = evalPoly(coef_g, m);
                    if (g_m == 0) {
                        t_cand = m;
                        break;
                    } else if (g_m < 0) {
                        a = m;
                    } else {
                        b = m;
                    }
                }
                p_cand = getPointFromControlPoints(control_points_rel, t_cand);
                dist_cand = p_cand.norm();
                if(dist_cand < dist_closest){
                    closest_point = p_cand;
                    dist_closest = dist_cand;
//                        t_normalized_closest = t_cand;
                    isClosestPointInSection = true;
                }
            }
        }
        if(not isClosestPointInSection){
            if(control_points_rel[0].norm() < control_points_rel[n].norm()){
                closest_point = control_points_rel[0];
//                    t_normalized_closest = 0;
            }
            else{
                closest_point = control_points_rel[n];
//                    t_normalized_closest = 1;
            }
        }

        return closest_point.norm();
//            // Exception handle for first segment
//            if(m == 0 && p_closest.norm() < d[0] - SP_EPSILON_FLOAT){
//                p_closest = control_points_rel[0];
//            }
    }

    static void buildBernsteinBasis(int n, Eigen::MatrixXd& B, Eigen::MatrixXd& B_inv) {
        B = Eigen::MatrixXd::Zero(n + 1, n + 1);
        for(int i = 0; i < n + 1; i++){
            for(int j = 0; j < n + 1; j++){
                if(j >= i){
                    B(i,j) = nChoosek(n, i) * nChoosek(n-i, n-j) * pow(-1, j-i);
                }
                else{
                    B(i,j) = 0;
                }
            }
        }
        B_inv = B.inverse();
    }
}
#endif //LSC_PLANNER_BERNSTEIN_TRAJECTORY_HPP
