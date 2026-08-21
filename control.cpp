#include<stdafx.h>

std::vector<char> Control::control_order{'0', '0'};
PositionVector Control::initial_momentum;
PositionVector Control::target_momentum;
PositionVector Control::rotation_momentum;
double Control::target_time;
double Control::gap;
std::vector<PositionVector> Control::rotation_momentums(3);
std::vector<double> Control::target_times(3);
PositionVector Control::rotation_axis;
PositionVector Control::slew_speedup_velocity;
PositionVector Control::slew_dump_velocity;
PositionVector Control::slew_plateau_velocity;
double Control::slew_speedup_time;
double Control::slew_plateau_time;
double Control::slew_dump_time;
Time Control::checkpoint_time;
Satellite Control::checkpoint_sat;

PositionVector Control::u(double t)
{
    // По принципу максимума Понтрягина с минимизацией u^2 получается такое соотношение    
    return (6.0*(2.0*t-target_time)*rotation_momentum - 2.0*(3.0*t-target_time)*target_momentum - 2.0*(3.0*t-2.0*target_time)*initial_momentum) / target_time / target_time;
}

PositionVector Control::u2(double t)
{
    PositionVector a = (12.0 * rotation_momentum - 6.0 * (target_momentum + initial_momentum)) / target_time / target_time;
    PositionVector b = (6.0 * rotation_momentum - 2.0 * target_momentum - 4.0 * initial_momentum) / target_time;
    return a * t * t * t / 6.0 - b * t * t / 2.0;
}

void Control::check_angles(const Matrix& r, double& psi, double& theta, double& phi)
{
    double psi2, phi2, theta2;

    if (fabs(r[2][2] - 1.0) < 1e-4) // theta = 0
    {
        theta2 = 0.0;
        psi2 = 0.0;
        phi2 = std::atan2(r[1][0], r[0][0]); 
    }
    else if (fabs(r[2][2] + 1.0) < 1e-4) // theta = 180
    {
        theta2 = -1.0 * M_PI;
        psi2 = 0.0;
        phi2 = std::atan2(r[1][0], r[0][0]); 
    }
    else if (fabs(r[2][2]) < 1e-4) // theta = -90 | +90
    {
        psi2 = std::atan2(-1.0 * r[0][2], r[1][2]);
        phi2 = std::atan2(-1.0 * r[2][0], -1.0 * r[2][1]);
        theta2 = -1.0 * M_PI_2; //std::asin(gtl[2][0] / std::sin(psi));
    }
    else
    {
        theta2 = -1.0 * std::acos(r[2][2]);
        phi2 = std::atan2(-1.0 * r[2][0], -1.0 * r[2][1]);
        psi2 = std::atan2(-1.0 * r[0][2], r[1][2]);
    }

    double summ = std::fabs(psi) + std::fabs(phi) + std::fabs(theta);
    double summ2 = std::fabs(psi2) + std::fabs(phi2) + std::fabs(theta2);

    if (summ > summ2)
    {
        phi = phi2;
        psi = psi2;
        theta = theta2;
    }

}

//void Control::defactorTarget(const Matrix& I, const boost::math::quaternion<double>& initial, const boost::math::quaternion<double>& target)
void Control::defactorTarget(const Matrix& I, const Quaternion& initial, const Quaternion& target)
{
    double phi, theta, psi;
    if (target == initial)
    {
        phi = 0.0;
        theta = 0.0;
        psi = 0.0;
    }
    else
    {
        // REVISE if some of the angles are 0 then needed less targets
        //Quaternion control_quat = initial.get_inverse() * target;
        // CRUTCH!!!
        //control_quat.set_x(-1.0 * control_quat.get_x());
        //control_quat.set_y(-1.0 * control_quat.get_y());
        //control_quat.set_z(-1.0 * control_quat.get_z());

        Matrix init = initial.to_matrix();
        Matrix end = target.to_matrix();

        Matrix gtl = init.transpose() * end;
        
        // Matrix gtl = control_quat.to_matrix();
        /*
        double w, x, y, z;
        w = control_quat.get_w();
        x = control_quat.get_x();
        y = control_quat.get_y();
        z = control_quat.get_z();
        */
        /* // Quaternion -> angles (no Matrix)
        if ((fabs(x) < 1e-4) && (fabs(y) < 1e-4)) // theta = 0
        {
            psi = 0.0;
            phi = 2.0 * std::atan2(z, w);
            theta = 0.0;
            if (phi > M_PI) phi -= 2.0 * M_PI;
            else if (phi < -1.0 * M_PI) phi += 2.0 * M_PI;
        }
        else if ((fabs(w) < 1e-4) && (fabs(z) < 1e-4)) // theta = 180
        {
            psi = 0.0;
            theta = M_PI;
            phi = 2.0 * std::atan2(y, x);
            if (phi > M_PI) phi -= 2.0 * M_PI;
            else if (phi < -1.0 * M_PI) phi += 2.0 * M_PI;
        }
        else
        {
            psi = std::atan2(z, w) + std::atan2(y, x);
            if (psi > M_PI) psi -= 2.0 * M_PI;
            else if (psi < -1.0 *  M_PI) psi += 2.0 * M_PI;

            phi = std::atan2(z, w) - std::atan2(y, x);
            if (phi > M_PI) phi -= 2.0 * M_PI;
            else if (phi < -1.0 * M_PI) phi += 2.0 * M_PI;

            theta = 2.0 * std::atan2(x / std::cos((phi - psi) / 2.0), w / std::cos((phi + psi) / 2.0));
            if (theta > M_PI) theta -= 2.0 * M_PI;
            else if (theta < -1.0 * M_PI) theta += 2.0 * M_PI;

        }
        */

        // Z - X - Z rotation
        
        if (fabs(gtl[2][2] - 1.0) < 1e-4) // theta = 0
        {
            theta = 0.0;
            psi = 0.0;
            phi = std::atan2(gtl[1][0], gtl[0][0]); 
        }
        else if (fabs(gtl[2][2] + 1.0) < 1e-4) // theta = 180
        {
            theta = M_PI;
            psi = 0.0;
            phi = std::atan2(gtl[1][0], gtl[0][0]); 
        }
        else if (fabs(gtl[2][2]) < 1e-4) // theta = -90 | +90
        {
            psi = std::atan2(gtl[0][2], -1.0 * gtl[1][2]);
            phi = std::atan2(gtl[2][0], gtl[2][1]);
            theta = M_PI_2; //std::asin(gtl[2][0] / std::sin(psi));
        }
        else
        {
            theta = std::acos(gtl[2][2]);
            phi = std::atan2(gtl[2][0], gtl[2][1]);
            psi = std::atan2(gtl[0][2], -1.0 * gtl[1][2]);
        }
        
        

        // Yaw - Pitch - Roll rotation: R = R_z(phi) * R_y(theta) * R_x(psi)
        /*
        if (fabs(gtl[2][0]) < 1e-4) // theta = 0 | 180
        {
            psi = 0.0;
            theta = 0.0;
            phi = std::atan2(gtl[2][1], gtl[2][2]);
        }
        else if (fabs(fabs(gtl[2][0]) - 1.) < 1e-4) // theta = +90 | -90
        {
            theta = std::asin(-1.0 * gtl[2][0]);
            psi = 0.0;
            phi = std::atan2(gtl[0][1], gtl[0][2]);
        }
        else
        {
            theta = std::asin(-1.0 * gtl[2][0]);
            psi = std::atan2(gtl[1][0], gtl[0][0]);
            phi = std::atan2(gtl[2][1], gtl[2][2]);
        }
        */
        std::cout << "Angles were: " << phi * 180.0 / M_PI << '\t' << theta * 180.0 / M_PI << '\t' << psi * 180.0 / M_PI << std::endl;
        check_angles(gtl, psi, theta, phi);
        std::cout << "Angles became: " << phi * 180.0 / M_PI << '\t' << theta * 180.0 / M_PI << '\t' << psi * 180.0 / M_PI << std::endl;

    }
    //std::cout << phi * 180.0 / M_PI << '\t' << theta * 180.0 / M_PI << '\t' << psi * 180.0 / M_PI << std::endl;
    
    /*
    int k = 0;
    if (fabs(psi) > 1e-4) k++;
    if (fabs(theta) > 1e-4) k++;
    if (fabs(phi) > 1e-4) k++;

    target_times[0] = 0.0; target_times[1] = 0.0; target_times[2] = 0.0;

    if (fabs(psi) > 1e-4)
        target_times[0] = std::floor(target_time / (double)k);
    if (fabs(theta) > 1e-4)
        target_times[1] = std::floor(target_time / (double)k);
    if (fabs(phi) > 1e-4)
        target_times[2] = target_time - target_times[0] - target_times[1];
   */

    double total_angle = std::fabs(psi) + std::fabs(theta) + std::fabs(phi);

    target_times[0] = std::floor(target_time * std::fabs(psi) / total_angle);
    target_times[1] = std::floor(target_time * std::fabs(theta) / total_angle);
    target_times[2] = target_time - target_times[0] - target_times[1];
   
    rotation_momentums[0] = mul(I, PositionVector({0.0, 0.0, psi / target_times[0]}));
    rotation_momentums[1] = mul(I, PositionVector({theta / target_times[1], 0.0, 0.0}));
    rotation_momentums[2] = mul(I, PositionVector({0.0, 0.0, phi / target_times[2]}));
    
    /*
    rotation_momentums[0] = mul(I, PositionVector({phi / target_times[0], 0.0, 0.0}));
    rotation_momentums[1] = mul(I, PositionVector({0.0, 0.0, theta / target_times[1]}));
    rotation_momentums[2] = mul(I, PositionVector({psi / target_times[2], 0.0, 0.0}));
    */
    std::cout << "Defactorisation: " << target_times[0] << '\t' << target_times[1] << '\t' << target_times[2] << std::endl;

    target_time = target_times[0];
    rotation_momentum = rotation_momentums[0];
}

int Control::setRotationFromQuat(Satellite& sat, const Quaternion& init, const Quaternion& target, const double gap, const double step, const PositionVector& initial_momentum, const PositionVector& torque)
{
    std::cout << init << '\t' << target << std::endl;
    Quaternion rot = init.get_inverse() * target;
    double half_angle = std::acos(rot.get_w());
    double sinus = std::sin(half_angle);

    rotation_axis[0] = rot.get_x() / sinus;
    rotation_axis[1] = rot.get_y() / sinus;
    rotation_axis[2] = rot.get_z() / sinus;

    double angle = 2.0 * half_angle;
    if (std::fabs(angle) > M_PI)
    {
        angle = angle - 2.0 * M_PI;
    }
    
    rotation_momentum = angle / target_time * mul(sat.getInertiaTensor(), rotation_axis);
    std::cout << "axis: " << rotation_axis << '\n' << "Angle: " << angle << '\n' << "Momentum: " << rotation_momentum << std::endl;

    double min_slew_time = Control::getMinSlewTime(sat, angle, rotation_axis, step, initial_momentum, torque);
    std::cout << "Min time: " <<  min_slew_time << std::endl;

    sat.setTargetDuration(min_slew_time);

    if (gap >= min_slew_time)
        Control::distributeTimeForSlew(sat, gap, angle, rotation_axis, step);
    else
    {
        // std::cout << "Too little time to perform rotation from " << init << " to " << target << std::endl;
        std::cerr << "\033[33m#9 Need at least " << min_slew_time << " seconds to perform rotation, but only " <<  gap << " given \033[0m" << std::endl;
        return 1;
    }

    return 0;


}

void Control::cancelDefactorisation()
{
    target_times[0] = 0;
}

void Control::setInitialMomentum(const PositionVector& im)
{
    initial_momentum = im;
}
void Control::setRotationMomentum(const PositionVector& rm)
{
    rotation_momentum = rm;
}

// REVISE : scan can be performed only around Satellite frame axis <=> tm should be of signature (1, 0, 0) or (0, 1, 0) or (0, 0, 1)
void Control::setTargetMomentum(const PositionVector& tm)
{
    target_momentum = tm;
}
void Control::setTargetTime(const double tt)
{
    target_time = tt;
    target_times[0] = 0.0; target_times[1] = 0.0; target_times[2] = 0.0;
}
void Control::setGap(const double g)
{
    gap = g;
}
void Control::setControlOrder(char first, char second)
{
    control_order[0] = first;
    control_order[1] = second;
}

std::vector<char> Control::getControlOrder()
{
    return control_order;
}
//PositionVector Control::getAngvelFromQuaternion(const boost::math::quaternion<double>& initial, const boost::math::quaternion<double>& target)
PositionVector Control::getAngvelFromQuaternion(const Quaternion& initial, const Quaternion& target)
{
    Quaternion control_quat = initial.get_inverse() * target;

    // Rotation angle
	//double angle = 2.0 * std::acos(control_quat.R_component_1());
    double angle = 2.0 * std::acos(control_quat.get_scalar());
	// if (angle >= M_PI) { angle -= 2.0*M_PI;  control_quat = -1.0 * control_quat; } // so that rotation is never more than 180 degrees -> always performable in one time-step

	// Rotation Axis
	//PositionVector angular_velocity = PositionVector({ control_quat.R_component_2(), control_quat.R_component_3(), control_quat.R_component_4() });
    PositionVector angular_velocity = control_quat.get_vector();
	angular_velocity = angular_velocity * angle / angular_velocity.norm() / target_time;
    std::cout << angle << '\t' << angular_velocity << std::endl;
    return angular_velocity;
}

PositionVector Control::beforeTarget(double current_time, double step, char system)
{
    if (target_times[0] != 0) // defactorization happened
    {
        if (current_time >= target_times[0] + target_times[1])
        {
            current_time -= target_times[0];
            current_time -= target_times[1];
            rotation_momentum = rotation_momentums[2];
            target_time = target_times[2];
        }
        else if (current_time >= target_times[0])
        {
            current_time -= target_times[0];
            rotation_momentum = rotation_momentums[1];
            target_time = target_times[1];
        }
    }
    if (system == 'g')
    {
        //return (u2(current_time + step) - u2(current_time)) / step;
        return rotation_momentum;
    }
    else
    {
        return (u(current_time) + u(current_time + step)) / 2.0 / step;
    }
}

PositionVector Control::performSlew(const Satellite& sat, double current_time, double step, char system)
{
    auto angles = sat.getReactionWheelsBlock()[0][0].getAngles();
    auto apex = sat.getReactionWheelsBlock()[0][0].getApex();

    // разгон
    if (current_time < slew_speedup_time)
    {
        return Control::combineReactionWheelsBlockMomentum(slew_speedup_velocity * current_time / step, angles, apex);
    }
    // плато
    else if (current_time <= slew_speedup_time + slew_plateau_time)
    {
        //return Control::combineReactionWheelsBlockMomentum(current_momentums, angles, apex);
        return Control::combineReactionWheelsBlockMomentum(slew_plateau_velocity, angles, apex);
    }
    // торможение
    else
    {
        return Control::combineReactionWheelsBlockMomentum(slew_plateau_velocity - slew_dump_velocity * (current_time - slew_speedup_time - slew_plateau_time) / step, angles, apex);
    }
}

PositionVector Control::redistributeCompensationMomentum(const PositionVector& target_mom, const PositionVector& angles, const std::string& apex)
{
    /*
        If control system is 4 reaction wheels. P4 is found from sum((pi-pi0)^2) -> min. Check MATLAB flywheel_equations.m
    

    double p10 = init_mom[0];
    double p20 = init_mom[1];
    double p30 = init_mom[2];
    double p40 = init_mom[3];
    double px = target_mom[0];
    double py = target_mom[1];
    double pz = target_mom[2];


    double p4 = (
    p40 - 
    (p30 - (py*cos_beta*sin_alpha + pz*cos_beta*sin_alpha - py*sin_alpha*sin_beta + pz*sin_alpha*sin_beta + px*cos_alpha)/(2.0*sin_alpha*cos_alpha)) + 
    (p20 + (pz*cos_beta - py*sin_beta)/(cos_alpha)) - 
    (p10 + (py*cos_beta*sin_alpha - pz*cos_beta*sin_alpha + py*sin_alpha*sin_beta + pz*sin_alpha*sin_beta - px*cos_alpha)/(2.0*cos_alpha*sin_alpha))
    ) / 4.0;

    double p1 = -(py*cos_beta*sin_alpha - pz*cos_beta*sin_alpha + py*sin_alpha*sin_beta + pz*sin_alpha*sin_beta - px*cos_alpha) / (2.0*cos_alpha*sin_alpha) - p4;
    double p2 = (py*sin_beta - pz*cos_beta) / cos_alpha + p4;
    double p3 = (py*cos_beta*sin_alpha + pz*cos_beta*sin_alpha - py*sin_alpha*sin_beta + pz*sin_alpha*sin_beta + px*cos_alpha) / (2.0*sin_alpha*cos_alpha) - p4;
    */

    /* sum(pi^2) -> min  solved with solve_test2.py*/
    
    double px, py, pz;

    if (apex == "X")
    {
        px = target_mom[0];
        py = target_mom[1];
        pz = target_mom[2];
    }
    else if (apex == "Y")
    {
        px = target_mom[1];
        py = target_mom[2];
        pz = target_mom[0];
    }
    else if (apex == "Z")
    {
        px = target_mom[2];
        py = target_mom[0];
        pz = target_mom[1];
    }

    if (angles.length() == 4)
    {
        double sin_alpha, cos_alpha, sin_beta, cos_beta;
        sin_alpha = angles[0];
        cos_alpha = angles[1];
        sin_beta = angles[2];
        cos_beta = angles[3];
        double p1 = px / (4.0 * sin_alpha) - py * cos_beta / (2.0 * cos_alpha) - pz * sin_beta / (2.0 * cos_alpha);
        double p2 = px / (4.0 * sin_alpha) + py * sin_beta / (2.0 * cos_alpha) - pz * cos_beta / (2.0 * cos_alpha);
        double p3 = px / (4.0 * sin_alpha) + py * cos_beta / (2.0 * cos_alpha) + pz * sin_beta / (2.0 * cos_alpha);
        double p4 = px / (4.0 * sin_alpha) - py * sin_beta / (2.0 * cos_alpha) + pz * cos_beta / (2.0 * cos_alpha);

        return PositionVector({p1, p2, p3, p4});
    }

    else if (angles.length() == 8)
    {
        double sa, ca, sb, cb, sa2, ca2, sb2, cb2;
        sa = angles[0];
        ca = angles[1];
        sb = angles[2];
        cb = angles[3];
        sa2 = angles[4];
        ca2 = angles[5];
        sb2 = angles[6];
        cb2 = angles[7];

        double p1 = px * sa / (4.0 * (sa*sa + sa2*sa2)) - py * ca * cb / (2.0*(ca*ca + ca2*ca2)) - pz * ca * sb / (2.0 * (ca*ca + ca2*ca2));
        double p2 = px * sa / (4.0 * (sa*sa + sa2*sa2)) + py * ca * sb / (2.0*(ca*ca + ca2*ca2)) - pz * ca * cb / (2.0 * (ca*ca + ca2*ca2));
        double p3 = px * sa / (4.0 * (sa*sa + sa2*sa2)) + py * ca * cb / (2.0*(ca*ca + ca2*ca2)) + pz * ca * sb / (2.0 * (ca*ca + ca2*ca2));
        double p4 = px * sa / (4.0 * (sa*sa + sa2*sa2)) - py * ca * sb / (2.0*(ca*ca + ca2*ca2)) + pz * ca * cb / (2.0 * (ca*ca + ca2*ca2));
        double p5 = px * sa2 / (4.0 * (sa*sa + sa2*sa2)) - py * ca2 * cb2 / (2.0*(ca*ca + ca2*ca2)) - pz * ca2 * sb2 / (2.0 * (ca*ca + ca2*ca2));
        double p6 = px * sa2 / (4.0 * (sa*sa + sa2*sa2)) + py * ca2 * sb2 / (2.0*(ca*ca + ca2*ca2)) - pz * ca2 * cb2 / (2.0 * (ca*ca + ca2*ca2));
        double p7 = px * sa2 / (4.0 * (sa*sa + sa2*sa2)) + py * ca2 * cb2 / (2.0*(ca*ca + ca2*ca2)) + pz * ca2 * sb2 / (2.0 * (ca*ca + ca2*ca2));
        double p8 = px * sa2 / (4.0 * (sa*sa + sa2*sa2)) - py * ca2 * sb2 / (2.0*(ca*ca + ca2*ca2)) + pz * ca2 * cb2 / (2.0 * (ca*ca + ca2*ca2));

        return PositionVector({p1, p2, p3, p4, p5, p6, p7, p8});
    }

    std::cerr << "\033[31m#21 NO PLAN FOR REDISTRIBUTING " << angles.length() << " REACTION WHEELS \033[0m" << std::endl;
    throw std::runtime_error("");
}

PositionVector Control::combineReactionWheelsBlockMomentum(const PositionVector& momentums, const PositionVector& angles, const std::string& apex)
{
    PositionVector rwb_momentums;

    if (momentums.length() == 4)
    {
        double sin_alpha = angles[0];
        double cos_alpha = angles[1];
        double sin_beta = angles[2];
        double cos_beta = angles[3];
        
        Matrix a(3,4,
        {
            {sin_alpha, sin_alpha, sin_alpha, sin_alpha},
            {-1.0*cos_alpha*cos_beta, cos_alpha*sin_beta, cos_alpha*cos_beta, -1.0*cos_alpha*sin_beta}, 
            {-1.0*cos_alpha*sin_beta, -1.0*cos_alpha*cos_beta, cos_alpha*sin_beta, cos_alpha*cos_beta}
        });
        rwb_momentums = mul(a, momentums);
    }
    else if (momentums.length() == 8)
    {
        double sa, ca, sb, cb, sa2, ca2, sb2, cb2;
        sa = angles[0];
        ca = angles[1];
        sb = angles[2];
        cb = angles[3];
        sa2 = angles[4];
        ca2 = angles[5];
        sb2 = angles[6];
        cb2 = angles[7];

        Matrix a(3,8,
        {
            {sa, sa, sa, sa, sa2, sa2, sa2, sa2},
            {-1.0*ca*cb, ca*sb, ca*cb, -1.0*ca*sb, -1.0*ca2*cb2, ca2*sb2, ca2*cb2, -1.0*ca2*sb2},
            {-1.0*ca*sb, -1.0*ca*cb, ca*sb, ca*cb, -1.0*ca2*sb2, -1.0*ca2*cb2, ca2*sb2, ca2*cb2}
        });
        rwb_momentums = mul(a, momentums);
    }
    else
    {
        std::cerr << "\033[31m#21 NO PLAN FOR COMBINING " << momentums.length() << " REACTION WHEELS\033[0m" << std::endl;
        throw std::runtime_error("");
    }

    if (apex == "X")
    {
        return rwb_momentums;
    }
    else if (apex == "Y")
    {
        return PositionVector({rwb_momentums[2], rwb_momentums[0], rwb_momentums[1]});
    }
    else if (apex == "Z")
    {
        return PositionVector({rwb_momentums[1], rwb_momentums[2], rwb_momentums[0]});
    }
    else
    {
        std::cerr << "\033[31m#11_apex Impossible apex " << apex << "\033[0m" << std::endl;
        throw std::runtime_error("");
    }
}

void Control::setRotationAxis(const PositionVector& ra)
{
    rotation_axis = ra;
}

PositionVector Control::getRotationAxis()
{
    return rotation_axis;
}

double Control::getTargetTime()
{
    return std::max(target_time, target_times[0] + target_times[1] + target_times[2]);
}
double Control::getGap()
{
    return gap;
}
PositionVector Control::getRotationMomentum()
{
    return rotation_momentum;
}

Matrix Control::getTransformMatrixFromApex(const PositionVector& angles, const std::string& apex)
{
    double sin_alpha, cos_alpha, sin_beta, cos_beta;
    sin_alpha = angles[0];
    cos_alpha = angles[1];
    sin_beta = angles[2];
    cos_beta = angles[3];

    if (apex == "X")
    {
        return Matrix(4, 3, std::vector<std::vector<double>>{
            {1.0 / (4.0 * sin_alpha), -1.0 * cos_beta / (2.0 * cos_alpha),  -1.0 * sin_beta / (2.0 * cos_alpha)},
            {1.0 / (4.0 * sin_alpha), sin_beta / (2.0 * cos_alpha),         -1.0 * cos_beta / (2.0 * cos_alpha)},
            {1.0 / (4.0 * sin_alpha), cos_beta / (2.0 * cos_alpha),         sin_beta / (2.0 * cos_alpha)},
            {1.0 / (4.0 * sin_alpha), -1.0 * sin_beta / (2.0 * cos_alpha),  cos_beta / (2.0 * cos_alpha)}
        });
    }

    if (apex == "Y")
    {
        return Matrix(4, 3, std::vector<std::vector<double>>{
            {-1.0 * sin_beta / (2.0 * cos_alpha),   1.0 / (4.0 * sin_alpha), -1.0 * cos_beta / (2.0 * cos_alpha)},
            {-1.0 * cos_beta / (2.0 * cos_alpha),   1.0 / (4.0 * sin_alpha), sin_beta / (2.0 * cos_alpha)},
            {sin_beta / (2.0 * cos_alpha),          1.0 / (4.0 * sin_alpha), cos_beta / (2.0 * cos_alpha)},
            {cos_beta / (2.0 * cos_alpha),          1.0 / (4.0 * sin_alpha), -1.0 * sin_beta / (2.0 * cos_alpha)}
        });
    }

    if (apex == "Z")
    {
        return Matrix(4, 3, std::vector<std::vector<double>>{
            {-1.0 * cos_beta / (2.0 * cos_alpha),   -1.0 * sin_beta / (2.0 * cos_alpha),   1.0 / (4.0 * sin_alpha)},
            {sin_beta / (2.0 * cos_alpha),          -1.0 * cos_beta / (2.0 * cos_alpha),   1.0 / (4.0 * sin_alpha)},
            {cos_beta / (2.0 * cos_alpha),          sin_beta / (2.0 * cos_alpha),          1.0 / (4.0 * sin_alpha)},
            {-1.0 * sin_beta / (2.0 * cos_alpha),   cos_beta / (2.0 * cos_alpha),          1.0 / (4.0 * sin_alpha)}
        });
    }

    std::cerr << "\033[31m#23 No transofmation matrirx found for apex " << apex << "\033[0m" << std::endl;
    throw std::runtime_error("");
}

double Control::getMinSlewTime(const Satellite& sat, double angle, const PositionVector& axis, double step, const PositionVector& initial_momentum, const PositionVector& torque)
{
    double slew_time = 600.0;
    double sin_alpha, cos_alpha, sin_beta, cos_beta;
    auto reaction_wheels = sat.getReactionWheelsBlock();

    PositionVector angles = reaction_wheels[0][0].getAngles();
    double limit = reaction_wheels[0][0].getLimit(); // они все одинаковые
    sin_alpha = angles[0];
    cos_alpha = angles[1];
    sin_beta = angles[2];
    cos_beta = angles[3];

    Matrix transform = Control::getTransformMatrixFromApex(angles, reaction_wheels[0][0].getApex());

    PositionVector const_part = mul(transform * sat.getInertiaTensor(), angle * axis);
    //PositionVector torque_part = mul(transform, torque * slew_time - initial_momentum);
    PositionVector torque_part = mul(transform, PositionVector({0.0, 0.0, 0.0}));

    PositionVector rwbs = const_part / slew_time + torque_part;

    // all reaction wheels have mumentums in thier limit range
    bool condition = ((-1.0 * limit <= rwbs[0] and rwbs[0] <= limit) && (-1.0 * limit <= rwbs[1] and rwbs[1] <= limit) &&
        (-1.0 * limit <= rwbs[2] and rwbs[2] <= limit) && (-1.0 * limit <= rwbs[3] and rwbs[3] <= limit));

    // search for right boundary
    while (!condition)
    {
        slew_time *= 2.0;
        //torque_part = mul(transform, torque * slew_time - initial_momentum);
        torque_part = PositionVector({0.0, 0.0, 0.0, 0.0});
        rwbs = const_part / slew_time + torque_part;
        condition = ((-1.0 * limit <= rwbs[0] and rwbs[0] <= limit) && (-1.0 * limit <= rwbs[1] and rwbs[1] <= limit) &&
            (-1.0 * limit <= rwbs[2] and rwbs[2] <= limit) && (-1.0 * limit <= rwbs[3] and rwbs[3] <= limit));
    }
    double r = slew_time;

    // getting the closest left boundary
    while (condition)
    {
        slew_time /= 2.0;
        //torque_part = mul(transform, torque * slew_time - initial_momentum);
        torque_part = PositionVector({0.0, 0.0, 0.0, 0.0});
        rwbs = const_part / slew_time + torque_part;
        condition = ((-1.0 * limit <= rwbs[0] and rwbs[0] <= limit) && (-1.0 * limit <= rwbs[1] and rwbs[1] <= limit) &&
            (-1.0 * limit <= rwbs[2] and rwbs[2] <= limit) && (-1.0 * limit <= rwbs[3] and rwbs[3] <= limit));
    }
    double l = slew_time;

    // binary search for a minimal slew time
    while (r - l > step)
    {
        slew_time = (r + l) / 2.0;
        //torque_part = mul(transform, torque * slew_time - initial_momentum);
        torque_part = PositionVector({0.0, 0.0, 0.0, 0.0});
        rwbs = const_part / slew_time + torque_part;
        condition = ((-1.0 * limit <= rwbs[0] and rwbs[0] <= limit) && (-1.0 * limit <= rwbs[1] and rwbs[1] <= limit) &&
            (-1.0 * limit <= rwbs[2] and rwbs[2] <= limit) && (-1.0 * limit <= rwbs[3] and rwbs[3] <= limit));
        if (condition)
        {
            r = slew_time;
        }
        else
        {
            l = slew_time;
        }
    }

    slew_time = r;
    
    //torque_part = mul(transform, torque * slew_time - initial_momentum);
    torque_part = PositionVector({0.0, 0.0, 0.0, 0.0});
    rwbs = const_part / std::ceil(slew_time) + torque_part;

    double dump_speed, acc_speed, max_speed;
    double speedup_steps, dump_steps;
    
    dump_speed = reaction_wheels[0][0].getDumpSpeed();
    acc_speed = reaction_wheels[0][0].getAccSpeed();
    max_speed = std::max(std::max(rwbs[0], rwbs[1]), std::max(rwbs[2], rwbs[3]));

    // time for dump
    dump_steps = std::ceil(max_speed / dump_speed / step);
    
    // time for speedup
    speedup_steps = std::ceil(max_speed / acc_speed / step);
    
    // на какой угол повернётся аппарат за время разгона/торможения
    double angle_so_far = 0.0;
    PositionVector increment = rwbs / speedup_steps;
    PositionVector local_angvel;
    PositionVector momentums({0.0, 0.0, 0.0, 0.0});
    PositionVector moms3d;

    // разгон
    for (int i = 0; i < speedup_steps; i++)
    {
        // Это кинетический момент маховиков 
        moms3d = Control::combineReactionWheelsBlockMomentum(momentums, reaction_wheels[0][0].getAngles(), reaction_wheels[0][0].getApex());
        
        // Это угловая скорость КА local_angvel = w * axis
        local_angvel =  mul(sat.getInertiaTensor().inverse(), moms3d);

        angle_so_far += std::fabs(local_angvel[0] / axis[0] * step);

        momentums += increment;
    }
    //  торможение
    for (int i = 0; i < dump_steps; i ++)
    {
        moms3d = Control::combineReactionWheelsBlockMomentum(momentums, reaction_wheels[0][0].getAngles(), reaction_wheels[0][0].getApex());

        local_angvel =  mul(sat.getInertiaTensor().inverse(), moms3d);
        angle_so_far += std::fabs(local_angvel[0] / axis[0] * step);
        momentums += -1.0 * increment;
    }

    moms3d = Control::combineReactionWheelsBlockMomentum(rwbs, reaction_wheels[0][0].getAngles(), reaction_wheels[0][0].getApex());
    double angvel = mul(sat.getInertiaTensor().inverse(), moms3d)[0] / axis[0];

    double flat_time = (angle - angle_so_far) / angvel;
    // std::cout << std::ceil(r) << std::endl;
    // std::cout << "Min time for slew: " << speedup_steps * step + flat_time + dump_steps * step << std::endl;
    return speedup_steps * step + flat_time + dump_steps * step;
}

void Control::distributeTimeForSlew(const Satellite& sat, const double time, const double angle, const PositionVector& axis, const double step)
{
    double sin_alpha, cos_alpha, sin_beta, cos_beta;
    auto reaction_wheels = sat.getReactionWheelsBlock();
    
    auto apex = reaction_wheels[0][0].getApex();
    PositionVector angles = reaction_wheels[0][0].getAngles();
    double limit = reaction_wheels[0][0].getLimit(); // они все одинаковые
    sin_alpha = angles[0];
    cos_alpha = angles[1];
    sin_beta = angles[2];
    cos_beta = angles[3];

    //Matrix transform = Control::getTransformMatrixFromApex(angles, reaction_wheels[0][0].getApex());

    //Matrix const_part = transform * sat.getInertiaTensor();

    Matrix const_part = sat.getInertiaTensor();

    double speed, t_up, t1, t_down;

    speed = 3.0 * angle / 2.0 * time;
    auto rws = Control::redistributeCompensationMomentum(mul(const_part, speed * axis), angles, apex);

    int iteration = 1;
    while (std::max(std::max(std::fabs(rws[0]), std::fabs(rws[1])), std::max(std::fabs(rws[2]), std::fabs(rws[3]))) >= limit)
    {
        speed = (1.5 - 0.005 * (double)iteration) * angle / time;
        rws = Control::redistributeCompensationMomentum(mul(const_part, speed * axis), angles, apex);
        iteration++;
    }

    t1 = std::ceil(2.0 * angle / speed - time);
    speed = 2.0 * angle / (time + t1);
    rws = Control::redistributeCompensationMomentum(mul(const_part, speed * axis), angles, apex);

    t_up = std::ceil(std::max(std::max(std::fabs(rws[0]), std::fabs(rws[1])), std::max(std::fabs(rws[2]), std::fabs(rws[3]))) / reaction_wheels[0][0].getAccSpeed()) * step;
    t_down = std::ceil(std::max(std::max(std::fabs(rws[0]), std::fabs(rws[1])), std::max(std::fabs(rws[2]), std::fabs(rws[3]))) / reaction_wheels[0][0].getDumpSpeed()) * step;

    if (t_up + t_down < time - t1)
    {
        t_up = std::ceil((time - t1) / 2.0);
        t_down = time - t1 - t_up;
    }

    slew_speedup_velocity = rws / (int)(t_up / step);
    slew_dump_velocity = rws / (int)(t_down / step);
    slew_plateau_velocity = rws;
    slew_speedup_time = t_up;
    slew_plateau_time = t1;
    slew_dump_time = t_down;

    /*
    PositionVector momentums({0.0, 0.0, 0.0, 0.0});
    PositionVector moms3d, local_angvel;
    double angle_so_far = 0.0;

    // разгон
    for (int i = 0; i < (int)(t_up / step); i++)
    {
        // Это кинетический момент маховиков 
        moms3d = Control::combineReactionWheelsBlockMomentum(momentums, angles, apex);
        
        // Это угловая скорость КА local_angvel = w * axis
        local_angvel =  mul(sat.getInertiaTensor().inverse(), moms3d);

        angle_so_far += std::fabs(local_angvel[0] / axis[0] * step);

        momentums += slew_speedup_velocity;
    }

    std::cout << "Angle after speedup: " << angle_so_far << std::endl;
    //  торможение
    for (int i = 0; i < (int)(t_down / step); i++)
    {
        moms3d = Control::combineReactionWheelsBlockMomentum(momentums, angles, apex);

        local_angvel =  mul(sat.getInertiaTensor().inverse(), moms3d);
        angle_so_far += std::fabs(local_angvel[0] / axis[0] * step);
        momentums += -1.0 * slew_dump_velocity;
    }
    std::cout << "ANgle after dump: " << angle_so_far << std::endl;
    */
}

void Control::makeCheckpoint(const Time& t, const Satellite& s)
{
    std::cout << "\033[34mAt time " << t << " checkpoint created\033[0m" << std::endl;
    checkpoint_time = t;
    checkpoint_sat = s;
}

void Control::getCheckpoint(Time& t, Satellite& s)
{
    checkpoint_sat.setSetbackTime(t - checkpoint_time);
    t = checkpoint_time;
    s = checkpoint_sat;

    // this also means that given target failed -> isObserved = false

}

PositionVector Control::testingControl(const Matrix& I, double current_time, double step)
{
    // Делаем поворот относительно локальной Ox на 60 градусов, а потом относительно локальной Oz на 60 градусов. Каждый поворот -- 10 секунд.
    target_time = 600.0;
    /* Stupid rotation testing
    if (current_time < 600.0) // first rotation
    {
        rotation_momentum = mul(I, PositionVector({M_PI / 3.0 / 600.0, 0.0, 0.0}));
        // thursters
        //return (u(current_time) + u(current_time + step)) / 2.0 / step;
    }
    else if (current_time < 1200.0) // second rotation
    {
        rotation_momentum = mul(I, PositionVector({0.0, 0.0, M_PI / 3.0 / 600.0}));
        // thrusters
        //return (u(current_time - 600.0) + u(current_time - 600.0 + step)) / 2.0 / step;
    }
    else // third rotation
    {
        rotation_momentum = mul(I, PositionVector({0.0, M_PI / 3.0 / 600.0, 0.0}));
        // thrusters
        //return (u(current_time - 1200.0) + u(current_time - 1200.0 + step)) / 2.0 / step;
    }
    */

    // "Smart" rotation testing: M_PI / 3 over axis a = (1/sqrt(3), 1/sqrt(3), 1/sqrt(3))
    rotation_momentum = mul(I, PositionVector({M_PI / 3.0 / target_time / std::sqrt(3.0), M_PI / 3.0 / target_time / std::sqrt(3.0), M_PI / 3.0 / target_time / std::sqrt(3.0)}));

    // reaction wheels
    return rotation_momentum;
}