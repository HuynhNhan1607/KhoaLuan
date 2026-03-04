#ifndef MODEL_H_
#define MODEL_H_

#ifdef __cplusplus
extern "C" {
#endif

void Load_Model(const char* engine_path);

void Predict(const float* input_data, float* percent_nlos, float* delta_x, float* delta_y);

void Cleanup_Model();

#ifdef __cplusplus
}
#endif

#endif