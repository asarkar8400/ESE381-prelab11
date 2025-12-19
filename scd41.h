#ifndef SCD41_H
#define SCD41_H

#include <stdint.h>
#include <stdbool.h>

extern uint16_t co2_ppm; //set variables as external so other files can access them
extern float temperature_C;
extern float humidity_percent;

void TWI0_init(void); //function prototypes
void scd41_start_periodic_measurement(void);
bool scd41_data_ready(void);
void scd41_read_measurement(void);

#endif
