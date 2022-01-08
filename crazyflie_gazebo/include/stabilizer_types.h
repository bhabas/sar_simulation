#include <stdint.h>
#include <stdbool.h>

typedef struct attitude_s {
  uint32_t timestamp;  // Timestamp when the data was computed

  float roll;
  float pitch;
  float yaw;
} attitude_t;

/* Orientation as a quaternion */
typedef struct quaternion_s {
  uint32_t timestamp;

  union {
    struct {
      float q0;
      float q1;
      float q2;
      float q3;
    };
    struct {
      float x;
      float y;
      float z;
      float w;
    };
  };
} quaternion_t;

struct vec3_s {
  uint32_t timestamp; // Timestamp when the data was computed

  float x;
  float y;
  float z;
};

typedef struct vec3_s vector_t;
typedef struct vec3_s point_t;
typedef struct vec3_s velocity_t;
typedef struct vec3_s acc_t;


typedef struct state_s {
  attitude_t attitude;      // deg (legacy CF2 body coordinate system, where pitch is inverted)
  quaternion_t attitudeQuaternion;
  point_t position;         // m
  velocity_t velocity;      // m/s
  acc_t acc;                // Gs (but acc.z without considering gravity)
} state_t;


