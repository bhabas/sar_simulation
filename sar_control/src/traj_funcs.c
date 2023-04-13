#include "traj_funcs.h"


Trajectory_Type Traj_Type = NONE;
axis_direction axis;


float Traj_Activate[3] = {0.0f, 0.0f, 0.0f};
float s_0_t[3] = {0.0f, 0.0f, 0.0f};   // Traj Start Point [m]
float s_f_t[3] = {0.0f, 0.0f, 0.0f};   // Traj End Point [m]
float v_t[3] = {0.0f, 0.0f, 0.0f};     // Traj Vel [m/s]
float a_t[3] = {0.0f, 0.0f, 0.0f};     // Traj Accel [m/s^2]
float T[3] = {0.0f, 0.0f, 0.0f};       // Traj completion time [s]
float t_traj[3] = {0.0f, 0.0f, 0.0f};  // Traj time counter [s]

void point2point_Traj()
{

    for(int i = 0; i<3; i++)
    {
        // CALCULATE ONLY DESIRED TRAJECTORIES
        if(Traj_Activate[i] == 1.0f)
        {
            float t = t_traj[i];

            if(t_traj[i] <= T[i] && T[i] != 0.0f) // SKIP CALC IF ALREADY AT END POSITION
            {
                // CALCULATE TIME SCALING VALUE S(t)
                float s_t = (3*powf(t,2)/powf(T[i],2) - 2*powf(t,3)/powf(T[i],3));
                float ds_t = (6*t/powf(T[i],2) - 6*powf(t,2)/powf(T[i],3));
                float dds_t = (6/powf(T[i],2) - 12*t/powf(T[i],3));

                // CONVERT PATH VALUES X(S) TO TRAJECTORY VALUES X(S(t))
                float pos_val = s_0_t[i] +  s_t * (s_f_t[i] - s_0_t[i]);
                float vel_val = ds_t * (s_f_t[i] - s_0_t[i]);
                float acc_val = dds_t * (s_f_t[i] - s_0_t[i]);

                // UPDATE DESIRED STATE VECTORS
                set_vec_element(&x_d, i, pos_val);
                set_vec_element(&v_d, i, vel_val);
                set_vec_element(&a_d, i, acc_val);
            }
            else
            {
                // CONVERT PATH VALUES X(S) TO TRAJECTORY VALUES X(S(t))
                float pos_val = s_f_t[i];
                float vel_val = 0.0f;
                float acc_val = 0.0f;
                
                // UPDATE DESIRED STATE VECTORS
                set_vec_element(&x_d, i, pos_val);
                set_vec_element(&v_d, i, vel_val);
                set_vec_element(&a_d, i, acc_val);
            }

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