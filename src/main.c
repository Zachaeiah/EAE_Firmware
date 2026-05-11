#include <stdio.h>
#include <stdlib.h>
#include "Ztransform_fp.h"
#include "Ztransform_fx.h"

// Helper function prototypes
static float clampf(float value, float min, float max);
static zfix_t clampfx(zfix_t value, zfix_t min_value, zfix_t max_value);



/**
 * @brief clams float between a min and max value
 * 
 * @param value The value to be clamped
 * @param min The min value to cmap to
 * @param max The max value to camp to
 * @return clamped float 
 */
static float clampf(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief clams zfix_t between a min and max value
 * 
 * @param value The value to be clamped
 * @param min The min value to cmap to
 * @param max The max value to camp to
 * @return clamped zfix_t
 */
static zfix_t clampfx(zfix_t value, zfix_t min_value, zfix_t max_value)
{
    if (value > max_value) {
        return max_value;
    }

    if (value < min_value) {
        return min_value;
    }

    return value;
}

// Prototypes for the PID demo functions
void PID_floating(int argc, char **argv);

// Prototype for the fixed-point PID demo function
void PID_Fixed(int argc, char **argv);

/**
 * @brief Main function to run the PID demos
 * 
 * @param argc Number of command line arguments
 * @param argv Array of command line arguments
 * @return int 
 */
int main(int argc, char **argv) {

  printf("Running floating point PID demo");
  PID_floating(argc, argv);

  printf("Running fixed point PID demo");
  PID_Fixed(argc, argv);

  return 0;
}

void PID_floating(int argc, char **argv)
{
    

    const float Ts       = 0.001f; // time step of the system
    const float sim_time = 1.000f; // time to run the sim

    const uint32_t total_samples = (uint32_t)(sim_time / Ts);

    float position_setpoint = 90.0f;

    // allow user to specify setpoint as 1st argument
    if (argc >= 2) {
        position_setpoint = (float)atof(argv[1]);
    }

    // output csv file name
    const char *csv_name = "pid_floating_output.csv";

    // allow user to specify csv name as 2nd argument
    if (argc >= 3) {
        csv_name = argv[2];
    }

    char csv_path[256];

    snprintf(csv_path, sizeof(csv_path), 
             "output/%s", csv_name);

    // Numerator of plant diff equ
    const float f_plant_b[] = {0.00103889f, 0.00102138f, 0.00000000f};

    // Denominator  of planr diff equ
    const float f_plant_a[] = {1.00000000f, -1.94249371f, 0.94249371f, 0.00000000f};
    
    // get the number of coefficients
    const uint32_t f_plant_nb = sizeof(f_plant_b) / sizeof(f_plant_b[0]);
    const uint32_t f_plant_na = sizeof(f_plant_a) / sizeof(f_plant_a[0]);

    // create the plant filter
    ZFilter_fp *plant_t = ZFilter_fp_ctor(f_plant_b,
                                        f_plant_nb,
                                        f_plant_a,
                                        f_plant_na);
    // check plant filter was malloc and not NULL
    if (!plant_t) {
        printf("ERROR: Failed to create plant filter.\n");
        return;
    }

    // Numerator of PID diff equ
    const float f_pid_b[] = {0.20501000f, -0.20499000f};

    // Denominator  of PID diff equ
    const float f_pid_a[] = {1.00000000f, -1.00000000f};

    // get the number of coefficients
    const uint32_t f_pid_nb = sizeof(f_pid_b) / sizeof(f_pid_b[0]);
    const uint32_t f_pid_na = sizeof(f_pid_a) / sizeof(f_pid_a[0]);

    // create the PID filter
    ZFilter_fp *pid_t = ZFilter_fp_ctor(f_pid_b,
                                        f_pid_nb,
                                        f_pid_a,
                                        f_pid_na);
    
    // check PID filter was malloc and not NULL
    if (!pid_t) {
        printf("ERROR: Failed to create PID filter.\n");
        ZFilter_fp_dtor(plant_t);
        return;
    }

    // open the csv file
    FILE *csv_file = fopen(csv_path, "w");

    if (!csv_file) {
        printf("ERROR: Failed to open CSV file: %s\n", csv_path);
        ZFilter_fp_dtor(pid_t);
        ZFilter_fp_dtor(plant_t);
        return;
    }

    // write the header at the top of the csv
    fprintf(csv_file,
            "time,setpoint,measured_position,error,raw_voltage,voltage_command\n");

    // Actuator voltage limits.
    const float max_voltage =  12.0f;
    const float min_voltage = -12.0f;

    // Simulation state.
    float measured_position = 0.0f;
    float error             = 0.0f;
    float raw_voltage       = 0.0f;
    float voltage_command   = 0.0f;
    float time              = 0.0f;

    printf("\nPID Motor Position Demo\n");
    printf("Setpoint: %.3f\n", position_setpoint);
    printf("Sample Time: %.3f s\n", Ts);
    printf("Runtime:     %.3f s\n", sim_time);
    printf("Output saved to: %s\n\n", csv_path);

    for (uint32_t sample_count = 0; sample_count <= total_samples; sample_count++)
    {
        time = (float)sample_count * Ts;

        // get the feedback error
        error = position_setpoint - measured_position;

        // update the filter
        raw_voltage = ZFilter_fp_update(pid_t, error);

        // clamp the output of the filter to +-12
        voltage_command = clampf(raw_voltage, min_voltage, max_voltage);

        // update the plant witht the output of the PID filter
        measured_position = ZFilter_fp_update(plant_t, voltage_command);

        // write state to csv file
        fprintf(csv_file,
                "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                time,
                position_setpoint,
                measured_position,
                error,
                raw_voltage,
                voltage_command);
    }

    // close the csf file
    fclose(csv_file);

    // free the PID filter
    ZFilter_fp_dtor(pid_t);

    // free the Plant filter
    ZFilter_fp_dtor(plant_t);
}

void PID_Fixed(int argc, char **argv)
{


    const float Ts       = 0.001f; // time step of the system
    const float sim_time = 1.000f; // time to run the sim

    const uint32_t total_samples = (uint32_t)(sim_time / Ts);

    float position_setpoint_float = 90.0f;

    // allow user to specify setpoint as 1st argument
    if (argc >= 2) {
        position_setpoint_float = (float)atof(argv[1]);
    }

    // output csv file name
    const char *csv_name = "pid_fixed_output.csv";

    // allow user to specify csv name as 2nd argument
    if (argc >= 3) {
        csv_name = argv[2];
    }

    char csv_path[256];

    snprintf(csv_path,
            sizeof(csv_path),
            "output/%s",
            csv_name);

    const zfix_t position_setpoint = ZFIX_FROM_DOUBLE(position_setpoint_float);

    // copy the same plant Zfilter but convert to fixed point

    // Numerator of plant diff equ as fixed point
    const zfix_t f_plant_b[] = {
        ZFIX_FROM_DOUBLE(0.00103889),
        ZFIX_FROM_DOUBLE(0.00102138),
        ZFIX_FROM_DOUBLE(0.00000000)
    };

     // Denominator  of plant diff equ as fixed point
    const zfix_t f_plant_a[] = {
        ZFIX_FROM_DOUBLE(1.00000000),
        ZFIX_FROM_DOUBLE(-1.94249371),
        ZFIX_FROM_DOUBLE(0.94249371),
        ZFIX_FROM_DOUBLE(0.00000000)
    };

    // get the number of coefficients
    const uint32_t f_plant_nb = sizeof(f_plant_b) / sizeof(f_plant_b[0]);
    const uint32_t f_plant_na = sizeof(f_plant_a) / sizeof(f_plant_a[0]);

    // create the plant filter
    ZFilter_fx *plant_t = ZFilter_fx_ctor(f_plant_b,
                                            f_plant_nb,
                                            f_plant_a,
                                            f_plant_na);
    
    // check plant filter was malloc and not NULL
    if (!plant_t) {
        printf("ERROR: Failed to create fixed-point plant filter.\n");
        return;
    }

    // copy the same pid Zfilter but convert to fixed point

    // Numerator of PID diff equ
    const zfix_t f_pid_b[] = {
        ZFIX_FROM_DOUBLE(0.20501000),
        ZFIX_FROM_DOUBLE(-0.20499000)
    };

    // Denominator  of PID diff equ
    const zfix_t f_pid_a[] = {
        ZFIX_FROM_DOUBLE(1.00000000),
        ZFIX_FROM_DOUBLE(-1.00000000)
    };

    // get the number of coefficients
    const uint32_t f_pid_nb = sizeof(f_pid_b) / sizeof(f_pid_b[0]);
    const uint32_t f_pid_na = sizeof(f_pid_a) / sizeof(f_pid_a[0]);

    // create the PID filter
    ZFilter_fx *pid_t = ZFilter_fx_ctor(f_pid_b,
                                        f_pid_nb,
                                        f_pid_a,
                                        f_pid_na);
    
    // check PID filter was malloc and not NULL
    if (!pid_t) {
        printf("ERROR: Failed to create fixed-point PID filter.\n");
        ZFilter_fx_dtor(plant_t);
        return;
    }

    // open the csv file
    FILE *csv_file = fopen(csv_path, "w");

    if (!csv_file) {
        printf("ERROR: Failed to open CSV file: %s\n", csv_path);
        ZFilter_fx_dtor(pid_t);
        ZFilter_fx_dtor(plant_t);
        return;
    }

    // write the header at the top of the csv
    fprintf(csv_file,
            "time,setpoint,measured_position,error,raw_voltage,voltage_command\n");

    // Actuator voltage limits.
    const zfix_t max_voltage = ZFIX_FROM_DOUBLE(12.0);
    const zfix_t min_voltage = ZFIX_FROM_DOUBLE(-12.0);

    // Simulation state.
    zfix_t measured_position = 0;
    zfix_t error             = 0;
    zfix_t raw_voltage       = 0;
    zfix_t voltage_command   = 0;

    printf("\nFixed-Point PID Motor Position Demo\n");
    printf("Setpoint: %.3f\n", position_setpoint_float);
    printf("Sample Time: %.3f s\n", Ts);
    printf("Runtime:     %.3f s\n", sim_time);
    printf("Output saved to: %s\n\n", csv_path);

    for (uint32_t sample_count = 0; sample_count <= total_samples; sample_count++)
    {
        const float time = (float)sample_count * Ts;

        // get the feedback error
        error = position_setpoint - measured_position;

        // update the filter
        raw_voltage = ZFilter_fx_update(pid_t, error);

        // clamp the output of the filter to +-12
        voltage_command = clampfx(raw_voltage, min_voltage, max_voltage);

        // update the plant with the output of the PID filter
        measured_position = ZFilter_fx_update(plant_t, voltage_command);

        // write state to csv file
        fprintf(csv_file,
                "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                time,
                ZFIX_TO_DOUBLE(position_setpoint),
                ZFIX_TO_DOUBLE(measured_position),
                ZFIX_TO_DOUBLE(error),
                ZFIX_TO_DOUBLE(raw_voltage),
                ZFIX_TO_DOUBLE(voltage_command));
    }

    // close the csf file
    fclose(csv_file);

    // free the PID filter
    ZFilter_fx_dtor(pid_t);

    // free the Plant filter
    ZFilter_fx_dtor(plant_t);
}