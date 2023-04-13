#include "traj_funcs.h"


Trajectory_Type Traj_Type = NONE;
axis_direction axis;


struct traj_vec Traj_Activate = {0.0f, 0.0f, 0.0f};
struct traj_vec s_0_t = {0.0f, 0.0f, 0.0f};   // Traj Start Point [m]
struct traj_vec s_f_t = {0.0f, 0.0f, 0.0f};   // Traj End Point [m]
struct traj_vec v_t = {0.0f, 0.0f, 0.0f};     // Traj Vel [m/s]
struct traj_vec a_t = {0.0f, 0.0f, 0.0f};     // Traj Accel [m/s^2]
struct traj_vec T = {0.0f, 0.0f, 0.0f};       // Traj completion time [s]
struct traj_vec t_traj = {0.0f, 0.0f, 0.0f};  // Traj time counter [s]

void point2point_Traj()
{

    for(int i = 0; i<3; i++)
    {
        // CALCULATE ONLY DESIRED TRAJECTORIES
        if(Traj_Activate.idx[i] == 1.0f)
        {
            float t = t_traj.idx[i];

            if(t_traj.idx[i] <= T.idx[i] && T.idx[i] != 0.0f) // SKIP CALC IF ALREADY AT END POSITION
            {
                // CALCULATE TIME SCALING VALUE S(t)
                float s_t = (3*powf(t,2)/powf(T.idx[i],2) - 2*powf(t,3)/powf(T.idx[i],3));
                float ds_t = (6*t/powf(T.idx[i],2) - 6*powf(t,2)/powf(T.idx[i],3));
                float dds_t = (6/powf(T.idx[i],2) - 12*t/powf(T.idx[i],3));

                // CONVERT PATH VALUES X(S) TO TRAJECTORY VALUES X(S(t))
                float pos_val = s_0_t.idx[i] +  s_t * (s_f_t.idx[i] - s_0_t.idx[i]);
                float vel_val = ds_t * (s_f_t.idx[i] - s_0_t.idx[i]);
                float acc_val = dds_t * (s_f_t.idx[i] - s_0_t.idx[i]);

                // UPDATE DESIRED STATE VECTORS
                set_vec_element(&x_d, i, pos_val);
                set_vec_element(&v_d, i, vel_val);
                set_vec_element(&a_d, i, acc_val);
            }
            else
            {
                // CONVERT PATH VALUES X(S) TO TRAJECTORY VALUES X(S(t))
                float pos_val = s_f_t.idx[i];
                float vel_val = 0.0f;
                float acc_val = 0.0f;
                
                // UPDATE DESIRED STATE VECTORS
                set_vec_element(&x_d, i, pos_val);
                set_vec_element(&v_d, i, vel_val);
                set_vec_element(&a_d, i, acc_val);
            }

            // INCREMENT TIME COUNTER FOR TRAJECTORY CALCULATIONS
            t_traj.idx[i] += dt;
        }

    }


}

