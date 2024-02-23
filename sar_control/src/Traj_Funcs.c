#include "Traj_Funcs.h"


Trajectory_Type Traj_Type = NONE;
axis_direction axis;


bool Traj_Active[3] = {false,false,false};
float s_0_t[3] = {0.0f, 0.0f, 0.0f};   // Traj Start Point [m]
float s_f_t[3] = {0.0f, 0.0f, 0.0f};   // Traj End Point [m]
float v_t[3] = {0.0f, 0.0f, 0.0f};     // Traj Vel [m/s]
float a_t[3] = {0.0f, 0.0f, 0.0f};     // Traj Accel [m/s^2]
float j_t[3] = {5.0f, 5.0f, 5.0f};        // Jerk [m/s^3]
float T[3] = {0.0f, 0.0f, 0.0f};       // Traj completion time [s]
float t_traj[3] = {0.0f, 0.0f, 0.0f};  // Traj time counter [s]

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

            float t_jerk = a_t[i]/j_t[i]; // Time to reach max acceleration

            float v_max = 1.0f;
            float a_sign = (S >= 0) ? 1.0f : -1.0f; // Determine the direction of acceleration
            float t_acc = v_max / fabsf(a_t[i]);

            float s_acc = 0.5f * a_t[i] * fsqr(t_acc);
            float s_const = S - 2 * s_acc;
            float t_const = s_const / v_max;
            T[i] = 2 * t_acc + t_const;

        
            float pos_val = 0.0f;
            float vel_val = 0.0f;
            float acc_val = 0.0f;
        
            // printf("t_jerk: %f\n", t_jerk);
            // printf("t_acc: %f\n", t_acc);
            // printf("v_max: %f\n", v_max);

            printf("s_const: %.2f s_acc: %.2f S: %.2f\n", s_const, s_acc, S);

            if (a_sign*s_const > 0) // CONSTANT VELOCITY PROFILE
            {
                if (t <= t_acc)
                {
                    pos_val = s_0_t[i] + 0.5f * a_sign * a_t[i] * fsqr(t);
                    vel_val = a_sign * a_t[i] * t;
                    acc_val = a_sign * a_t[i];

                }
                else if (t_acc < t && t <= T[i] - t_acc)
                {
                    pos_val = s_0_t[i] + s_acc + a_sign * v_max * (t - t_acc);
                    vel_val = a_sign * v_max;
                    acc_val = 0.0f;
                }
                else if (T[i] - t_acc < t && t <= T[i])
                {
                    pos_val = s_f_t[i] - 0.5f * a_sign * a_t[i] * fsqr(T[i] - t);
                    vel_val = a_sign * (v_max - a_t[i] * (t - (T[i] - t_acc)));
                    acc_val = -a_sign * a_t[i];

                }
                else
                {
                    pos_val = s_f_t[i];
                    vel_val = 0.0f;
                    acc_val = 0.0f;
                }

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

void const_velocity_Traj()
{
   
    float t_x = v_t[0]/a_t[0];
    float t_z = v_t[2]/a_t[2];
    float t = t_traj[0];
     
    // X-ACCELERATION
    if(t < t_x) 
    {
        x_d.x = 0.5f*a_t[0]*t*t + s_0_t[0]; // 0.5*a_x*t^2 + x_0
        v_d.x = a_t[0]*t;  // a_x*t
        a_d.x = a_t[0];    // a_x

        x_d.z = s_0_t[2]; // z_0
        v_d.z = 0.0f;
        a_d.z = 0.0f;

    }

    // Z-ACCELERATION (CONSTANT X-VELOCITY)
    else if(t_x <= t && t < (t_x+t_z))
    {
        x_d.x = v_t[0]*t - fsqr(v_t[0])/(2.0f*a_t[0]) + s_0_t[0]; // vx*t - (vx/(2*ax))^2 + x_0
        v_d.x = v_t[0]; // vx
        a_d.x = 0.0f;

        x_d.z = 0.5f*a_t[2]*fsqr(t-t_x) + s_0_t[2]; // 0.5*az*t^2 + z_0
        v_d.z = a_t[2]*(t-t_x); // az*t
        a_d.z = a_t[2]; // az
    }

    // CONSTANT X-VELOCITY AND CONSTANT Z-VELOCITY
    else if((t_x+t_z) <= t )
    {
        x_d.x = v_t[0]*t - fsqr(v_t[0])/(2.0f*a_t[0]) + s_0_t[0]; // vx*t - (vx/(2*ax))^2 + x_0
        v_d.x = v_t[0]; // vx
        a_d.x = 0.0;

        x_d.z = v_t[2]*(t-t_x) - fsqr(v_t[2])/(2.0f*a_t[2]) + s_0_t[2]; // vz*t - (vz/(2*az))^2 + z_0
        v_d.z = v_t[2]; // vz
        a_d.z = 0.0f;
    }

    t_traj[0] += dt;
    
}

void const_velocity_GZ_Traj()
{
    float t = t_traj[0];
     
    // CONSTANT X-VELOCITY AND CONSTANT Z-VELOCITY
    x_d.x = v_t[0]*t + s_0_t[0]; // vx*t + x_0
    v_d.x = v_t[0]; // vx
    a_d.x = 0.0;

    x_d.y = v_t[1]*t + s_0_t[1]; // vy*t + y_0
    v_d.y = v_t[1]; // vy
    a_d.y = 0.0;

    x_d.z = v_t[2]*t + s_0_t[2]; // vz*t + z_0
    v_d.z = v_t[2]; // vz
    a_d.z = 0.0f;
    

    t_traj[0] += dt;

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