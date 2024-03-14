#include "Traj_Funcs.h"


Trajectory_Type Traj_Type = NONE;
axis_direction axis;


bool Traj_Active[3] = {false,false,false};
float s_0_t[3] = {0.0f, 0.0f, 0.0f};        // Traj Start Point [m]
float s_f_t[3] = {0.0f, 0.0f, 0.0f};        // Traj End Point [m]
float v_t[3] = {0.0f, 0.0f, 0.0f};          // Traj Vel [m/s]
float a_t[3] = {0.0f, 0.0f, 0.0f};          // Traj Accel [m/s^2]
float j_t[3] = {100.0f, 100.0f, 100.0f};    // Traj Jerk [m/s^3]
float T[3] = {0.0f, 0.0f, 0.0f};            // Traj completion time [s]
float t_traj[3] = {0.0f, 0.0f, 0.0f};       // Traj time counter [s]

void resetTraj_Vals(uint8_t axis)
{
    Traj_Active[axis] = false;
    s_0_t[axis] = 0.0f;
    s_f_t[axis] = 0.0f;
    v_t[axis] = 0.0f;
    a_t[axis] = 0.0f;
    T[axis] = 0.0f;
    t_traj[axis] = 0.0f;
}

void point2point_Traj()
{
    // ITERATE THROUGH EACH AXIS
    for(int i = 0; i<3; i++)
    {
        // CALCULATE ONLY DESIRED TRAJECTORIES
        if(Traj_Active[i] == true)
        {
            float t = t_traj[i];
            float S = s_f_t[i] - s_0_t[i];
            float a_sign = (S) >= 0 ? 1.0f : -1.0f; // Determine the direction of movement

            float v_max = sqrtf(0.5f * a_t[i] * fabsf(S));
            float t_acc = v_max / a_t[i];

            float s_acc = 0.5f * a_t[i] * fsqr(t_acc);
            float s_const = fabsf(S) - 2 * s_acc;
            float t_const = s_const / v_max;
            T[i] = 2 * t_acc + t_const;
        
            float pos_val = 0.0f;
            float vel_val = 0.0f;
            float acc_val = 0.0f;

            if (t <= t_acc)
            {
                acc_val = a_sign * a_t[i];
                vel_val = a_sign * a_t[i] * t;
                pos_val = a_sign * 0.5f * a_t[i] * fsqr(t);

            }
            else if (t_acc < t && t <= T[i] - t_acc)
            {
                acc_val = 0.0f;
                vel_val = v_max;
                pos_val = s_acc + v_max * (t - t_acc);

                acc_val *= a_sign;
                vel_val *= a_sign;
                pos_val *= a_sign;
            }
            else if (T[i] - t_acc < t && t <= T[i])
            {
                acc_val = -a_t[i];
                vel_val = v_max - a_t[i] * (t - (T[i] - t_acc));
                pos_val = s_acc + s_const + (s_acc - 0.5f * a_t[i] * fsqr(T[i] - t));

                acc_val *= a_sign;
                vel_val *= a_sign;
                pos_val *= a_sign;
            }
            else if (t > T[i])
            {
                acc_val = 0.0f;
                vel_val = 0.0f;
                pos_val = s_acc + s_const + s_acc;

                acc_val *= a_sign;
                vel_val *= a_sign;
                pos_val *= a_sign;

                Traj_Active[i] = false;
            }

            pos_val += s_0_t[i];

            // UPDATE DESIRED STATE VECTORS
            set_vec_element(&x_d, i, pos_val);
            set_vec_element(&v_d, i, vel_val);
            set_vec_element(&a_d, i, acc_val);
            
            

            // INCREMENT TIME COUNTER FOR TRAJECTORY CALCULATIONS
            t_traj[i] += dt;
        }

    }


}

void const_velocity_Traj()
{
    // ITERATE THROUGH EACH AXIS
    for(int i = 0; i<3; i++)
    {
        // CALCULATE ONLY DESIRED TRAJECTORIES
        if(Traj_Active[i] == true)
        {
            float t = t_traj[i];
            float t_jerk = a_t[i] / j_t[i];

            float v_j = 0.5f * j_t[i] * fsqr(t_jerk);
            float t_acc = (v_t[i] - 2.0f*v_j)/ a_t[i];

            float t_1 = t_jerk;
            float t_2 = t_jerk + t_acc;
            float t_3 = t_jerk + t_acc + t_jerk;

            float dds_1 = j_t[i]*t_1;
            float ds_1 = 0.5f*j_t[i]*fsqr(t_1);
            float s_1 = 1/6.0f*j_t[i]*pow(t_1,3);

            float dds_2 = a_t[i];
            float ds_2 = ds_1 + a_t[i]*(t_2-t_1);
            float s_2 = s_1 + ds_1*(t_2-t_1) + 0.5f*a_t[i]*fsqr(t_2-t_1);

            float dds_3 = a_t[i] - j_t[i]*(t_3-t_2);
            float ds_3 = ds_2 + a_t[i]*(t_3-t_2) - 0.5f*j_t[i]*fsqr(t_3-t_2);
            float s_3 = s_2 + ds_2*(t_3-t_2) + 0.5f*a_t[i]*fsqr(t_3-t_2) - 1/6.0f*j_t[i]*powf(t_3-t_2,3);
        
            float pos_val = 0.0f;
            float vel_val = 0.0f;
            float acc_val = 0.0f;

            if (i == 0)
            {
                T[0] = t_3;
            }

            if (i == 2)
            {
                t = t - T[0];
                if (t < 0.0f)
                {
                    t_traj[i] += dt;
                    continue;
                }
                
            }
            
            

            if (t <= t_1)
            {
                acc_val = j_t[i] * t;
                vel_val = 0.5f * j_t[i] * fsqr(t);
                pos_val = (1.0f/6.0f) * j_t[i] * powf(t,3.0f);
            }
            else if (t_1 < t && t <= t_2)
            {
                acc_val = a_t[i];
                vel_val = ds_1 + a_t[i]*(t-t_1);
                pos_val = s_1 + ds_1*(t-t_1) + 0.5f*a_t[i]*fsqr(t-t_1);
            }
            else if (t_2 < t && t <= t_3)
            {
                acc_val = a_t[i] - j_t[i]*(t - t_2);
                vel_val = ds_2 + a_t[i]*(t-t_2) - 0.5f*j_t[i]*powf(t-t_2,2);
                pos_val = s_2 + ds_2*(t-t_2) + 0.5f*a_t[i]*fsqr(t-t_2) - 1/6.0f*j_t[i]*powf(t-t_3,3);
            }
            else if (t_3 < t)
            {
                acc_val = 0.0f;
                vel_val = v_t[i];
                pos_val = s_3 + ds_3*(t-t_3);
            }

            pos_val += s_0_t[i];

            // UPDATE DESIRED STATE VECTORS
            set_vec_element(&x_d, i, pos_val);
            set_vec_element(&v_d, i, vel_val);
            set_vec_element(&a_d, i, acc_val);
            

            // INCREMENT TIME COUNTER FOR TRAJECTORY CALCULATIONS
            t_traj[i] += dt;
        }

    }

    
}

void const_velocity_GZ_Traj()
{
    // ITERATE THROUGH EACH AXIS
    for(int i = 0; i<3; i++)
    {
        // CALCULATE ONLY DESIRED TRAJECTORIES
        if(Traj_Active[i] == true)
        {
            float t = t_traj[i];
           
            float pos_val = 0.0f;
            float vel_val = 0.0f;
            float acc_val = 0.0f;


            // CONSTANT VELOCITY TRAJECTORIES W/O ACCELERATION
            acc_val = 0.0f;
            vel_val = v_t[i];
            pos_val = v_t[i] * t;

            // UPDATE RELATIVE TO STARTING POSITION
            pos_val += s_0_t[i];

            // UPDATE DESIRED STATE VECTORS
            set_vec_element(&x_d, i, pos_val);
            set_vec_element(&v_d, i, vel_val);
            set_vec_element(&a_d, i, acc_val);
            

            // INCREMENT TIME COUNTER FOR TRAJECTORY CALCULATIONS
            t_traj[i] += dt;
        }

    } 

}

void set_vec_element(struct vec *v, int index, float value) {

    switch (index) {
        case 0:
            v->x = value;
            break;
        case 1:
            v->y = value;
            break;
        case 2:
            v->z = value;
            break;
        default:
            // Handle invalid index if necessary
            break;
    }
}