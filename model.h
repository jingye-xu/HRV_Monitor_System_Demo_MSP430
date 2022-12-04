/*
 * model.h
 *
 *  Created on: 12/15/2021
 *      Author: jingye_lab_win
 */

#ifndef MODEL_H_
#define MODEL_H_

#define HRV_COUNT 120
#define HRV_L1 10
#define HRV_L2 10

#define MINSCALER 40
#define MAXSCALER 200

void PredictHR(float *input, float *output, int batch_len);
void forwardHR(float *input, float *output);

void PredictHRV(float *input, float *output, int batch_len);
void forwardHRV(float *input, float *output);
void calculateHRV(float *input, int length, float *output);

void DTHRV(float *input, float *output, int batch_len);
float DTMinMaxScaler(float input);

#endif /* MODEL_H_ */
