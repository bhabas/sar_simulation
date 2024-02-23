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
            float a_sign = (S) >= 0 ? 1.0f : -1.0f; // Determine the direction of movement

            float t_jerk = a_t[i]/j_t[i]; // Time to reach max acceleration

            float A = 0.5f*a_t[i];
            float B = 0.5f*j_t[i]*fsqr(t_jerk) + a_t[i]*t_jerk;
            float C = 0.5f*a_t[i]*fsqr(t_jerk) + 0.5*j_t[i]*powf(t_jerk,3) - 0.25f*S;
            float t_acc = (-B + sqrtf(B*B - 4*A*C))/(2.0f*A);


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


            float s_const = 0.5f*S;
            float t_const = s_const / ds_3;

            float t_4 = t_jerk + t_acc + t_jerk + t_const;
            float t_5 = t_jerk + t_acc + t_jerk + t_const + t_jerk;
            float t_6 = t_jerk + t_acc + t_jerk + t_const + t_jerk + t_acc;
            float t_7 = t_jerk + t_acc + t_jerk + t_const + t_jerk + t_acc + t_jerk;

            float dds_4 = 0.0f;
            float ds_4 = ds_3;
            float s_4 = s_3 + ds_3*(t_4-t_3);

            float dds_5 = -j_t[i]*(t_5-t_4);
            float ds_5 = ds_4 - 0.5f*j_t[i]*fsqr(t_5-t_4);
            float s_5 = s_4 + ds_4*(t_5-t_4) - 1/6.0f*j_t[i]*powf(t_5-t_4,3);

            float dds_6 = -a_t[i];
            float ds_6 = ds_5 - a_t[i]*(t_6-t_5);
            float s_6 = s_5 + ds_5*(t_6-t_5) - 0.5f*a_t[i]*fsqr(t_6-t_5);

        
            float pos_val = 0.0f;
            float vel_val = 0.0f;
            float acc_val = 0.0f;
        
            // printf("t_jerk: %f\n", t_jerk);
            // printf("t_acc: %f\n", t_acc);
            // printf("v_max: %f\n", v_max);

            if (t <= t_1)
            {
                acc_val = j_t[i] * t;
                vel_val = 0.5f * j_t[i] * fsqr(t);
                pos_val = 1/6.0f*j_t[i]*powf(t,3);
            }
            else if (t_1 < t && t <= t_2)
            {
                acc_val = a_t[i];
                vel_val = ds_1 + a_t[i]*(t - t_1);
                pos_val = s_1 + ds_1*(t-t_1) + 0.5f*a_t[i]*fsqr(t-t_1);

            }
            else if (t_2 < t && t <= t_3)
            {
                acc_val = a_t[i] - j_t[i]*(t - t_2);
                vel_val = ds_2 + a_t[i]*(t-t_2) - 0.5f*j_t[i]*powf(t-t_2,2);
                pos_val = s_2 + ds_2*(t-t_2) + 0.5f*a_t[i]*fsqr(t-t_2) - 1/6.0f*j_t[i]*powf(t-t_3,3);
            }
            else if (t_3 < t && t <= t_4)
            {
                acc_val = 0.0f;
                vel_val = ds_3;
                pos_val = s_3 + ds_3*(t-t_3);
            }
            else if (t_4 < t && t <= t_5)
            {
                acc_val = -j_t[i]*(t - t_4);
                vel_val = ds_4 - 0.5f*j_t[i]*fsqr(t-t_4);
                pos_val = s_4 + ds_4*(t-t_4) - 1/6.0f*j_t[i]*powf(t-t_4,3);
            }
            else if (t_5 < t && t <= t_6)
            {
                acc_val = -a_t[i];
                vel_val = ds_5 - a_t[i]*(t-t_5);
                pos_val = s_5 + ds_5*(t-t_5) - 0.5f*a_t[i]*fsqr(t-t_5);
            }
            else if (t_6 < t && t <= t_7)
            {
                acc_val = -a_t[i] + j_t[i]*(t-t_6);
                vel_val = ds_6 - a_t[i]*(t-t_6) + 0.5f*j_t[i]*fsqr(t-t_6);
                pos_val = s_6 + ds_6*(t-t_6) - 0.5f*a_t[i]*fsqr(t-t_6) + 1/6.0f*j_t[i]*powf(t-t_6,3);
            }
            else
            {
                acc_val = 0.0f;
                vel_val = 0.0f;
                pos_val = s_f_t[i];
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