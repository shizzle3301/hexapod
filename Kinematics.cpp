/**
 * @file Kinematics.cpp
 * @brief Full-body kinematics: body pose transforms, Jacobians
 */

#include "locomotion/Kinematics.hpp"
#include "utils/MathUtils.hpp"
#include <cmath>
#include <algorithm>

namespace hexapod {

Mat3 Kinematics::rpy_to_rotation(const Vec3& rpy) {
    return math::rpy_to_rot(rpy.x, rpy.y, rpy.z);
}

Vec3 Kinematics::body_to_world(const Vec3& p, const Vec3& body_pos,
                                const Vec3& body_rpy) {
    Mat3 R = rpy_to_rotation(body_rpy);
    return body_pos + R * p;
}

Vec3 Kinematics::world_to_body(const Vec3& p, const Vec3& body_pos,
                                const Vec3& body_rpy) {
    Mat3 R   = rpy_to_rotation(body_rpy);
    Mat3 Rt  = R.transposed();
    return Rt * (p - body_pos);
}

// ─────────────────────────────────────────────────────────────────────────────
bool Kinematics::solve_leg_ik(const Vec3& target, double coxa_len,
                               double femur_len, double tibia_len,
                               JointAngles& out)
{
    using namespace math;

    double coxa_angle = std::atan2(target.y, target.x);

    double reach_xy = std::sqrt(target.x * target.x + target.y * target.y)
                    - coxa_len;
    double dz = target.z;
    double d  = std::sqrt(reach_xy * reach_xy + dz * dz);

    double max_reach = femur_len + tibia_len;
    if (d > max_reach) return false;

    double cos_tibia = (femur_len * femur_len + tibia_len * tibia_len - d * d)
                     / (2.0 * femur_len * tibia_len);
    cos_tibia = clamp(cos_tibia, -1.0, 1.0);
    double tibia_angle = std::acos(cos_tibia) - PI;

    double alpha = std::atan2(dz, reach_xy);
    double cos_beta = (femur_len * femur_len + d * d - tibia_len * tibia_len)
                    / (2.0 * femur_len * d);
    cos_beta = clamp(cos_beta, -1.0, 1.0);
    double femur_angle = alpha - std::acos(cos_beta);

    out.coxa  = rad2deg(coxa_angle);
    out.femur = rad2deg(femur_angle);
    out.tibia = rad2deg(tibia_angle);
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
Vec3 Kinematics::solve_leg_fk(const JointAngles& a, double coxa_len,
                               double femur_len, double tibia_len)
{
    using namespace math;
    double cr = deg2rad(a.coxa), fr = deg2rad(a.femur), tr = deg2rad(a.tibia);

    double reach = coxa_len
                 + femur_len * std::cos(fr)
                 + tibia_len * std::cos(fr + tr);

    return {
        reach * std::cos(cr),
        reach * std::sin(cr),
        femur_len * std::sin(fr) + tibia_len * std::sin(fr + tr)
    };
}

// ─────────────────────────────────────────────────────────────────────────────
double Kinematics::stability_margin(const Vec3& cog,
                                     const std::vector<Vec3>& polygon)
{
    double min_dist = 1e9;
    int n = static_cast<int>(polygon.size());
    for (int i = 0; i < n; ++i) {
        const Vec3& a = polygon[i];
        const Vec3& b = polygon[(i + 1) % n];
        Vec3 edge = b - a;
        Vec3 to_p = cog - a;
        double len = std::sqrt(edge.x * edge.x + edge.y * edge.y);
        if (len < 1e-9) continue;
        double dist = std::abs(edge.x * to_p.y - edge.y * to_p.x) / len;
        min_dist = std::min(min_dist, dist);
    }
    return min_dist;
}

bool Kinematics::is_inside_support_polygon(const Vec3& cog,
                                            const std::vector<Vec3>& polygon)
{
    return math::point_in_convex_polygon(cog, polygon);
}

Vec3 Kinematics::compute_cog_projection(const std::array<Vec3, 6>& footholds,
                                         const Vec3& body_pos)
{
    (void)footholds;
    return {body_pos.x, body_pos.y, 0.0};
}

Mat3 Kinematics::leg_jacobian(const JointAngles& a, double coxa_len,
                               double femur_len, double tibia_len)
{
    (void)a; (void)coxa_len; (void)femur_len; (void)tibia_len;
    return Mat3::identity();
}

Vec3 Kinematics::force_to_torques(const Vec3& tip_force, const Mat3& jacobian)
{
    return jacobian.transposed() * tip_force;
}

Vec3 Kinematics::body_ik(const std::array<Vec3, 6>& footholds,
                          const Vec3& body_rpy, double body_height)
{
    Vec3 avg{0,0,0};
    for (auto& f : footholds) avg += f;
    avg = avg / 6.0;
    return {avg.x, avg.y, avg.z + body_height};
}

} // namespace hexapod
